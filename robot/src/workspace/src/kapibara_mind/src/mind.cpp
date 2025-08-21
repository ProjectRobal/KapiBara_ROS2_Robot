#include <experimental/simd>
#include <iostream>
#include <string_view>
#include <cmath>
#include <chrono>
#include <cstddef>
#include <iomanip>
#include <numeric>
#include <fstream>
#include <sys/stat.h>
#include <fcntl.h>
#include <algorithm>
#include <csignal> 
#include <random>
#include <signal.h>

#include <sqlite3.h>
#include <sqlite-vec.h>

#include <sensor_msgs/msg/point_cloud2.hpp>


#include <rclcpp/rclcpp.hpp>

// quanterion
#include <sensor_msgs/msg/imu.hpp>
// mean face embedded
#include <kapibara_interfaces/msg/face_embed.hpp>
// emotions, network will be triggered by emotions message
#include <kapibara_interfaces/msg/emotions.hpp>

#include <kapibara_interfaces/srv/stop_mind.hpp>


// encoders speed
#include <nav_msgs/msg/odometry.hpp>
// spectogram
#include <sensor_msgs/msg/image.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// twist message used for ros2 control
#include <geometry_msgs/msg/twist.hpp>

#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include "config.hpp"

#include "simd_vector.hpp"

#include "layer_kac.hpp"
 
#include "initializers/gauss.hpp"
#include "initializers/constant.hpp"
#include "initializers/uniform.hpp"
#include "initializers/hu.hpp"


#include "activation/sigmoid.hpp"
#include "activation/relu.hpp"
#include "activation/softmax.hpp"
#include "activation/silu.hpp"

#include "simd_vector_lite.hpp"

#include "layer_counter.hpp"

#include "arbiter.hpp"

#include "kapibara_sublayer.hpp"

#include "attention.hpp"

#include "shift_buffer.hpp"

#include "block_kac.hpp"

#include "common_types.hpp"

size_t snn::BlockCounter::BlockID = 0;

size_t snn::LayerCounter::LayerIDCounter = 0;

/*
    
    Now robot will collect points in 2D space into SQLite database, and assigns them to bags which will have id.

    Now there will be also Encoder AI model that will convert sensory data into specific bag id.

    The robot will use this bag id to retrieve points from database and use them to calculate forces in 2D space.

    We should decide whether point should go to bag or not. My idea is to keep track of past added points and  
    check distance between new and old point.

    How to generate embeddings for encoder? Use randomly generated values or go fully unsuprevised learning.
*/


using std::placeholders::_1;

using namespace std::chrono_literals;


#define MAX_LINEAR_SPEED (4.f)

#define MAX_ANGULAR_SPEED (400.f)



// Behavioral map size

#define STEP_SIZE (10.f)


#define LINEAR_FORCE_COF (10.f)

#define ANGULAR_FORCE_COF (50.f)



struct snapshot
{
    float image[90*90];
    float image_laplace[90*90];
    float spectogram[90*90];

    int32_t x;
    int32_t y;
};




class KapiBaraMind : public rclcpp::Node
{
    // collect snapshots of images, in some kind of buffers
   
    std::vector<point> map_points;

    number position[3];

    number orientation[4];

    snapshot current_snapshot;

    ShiftBuffer<snapshot,8> snapshots;

    sqlite3 *db;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr field_publisher;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orientation_subscription;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription;

    rclcpp::Subscription<kapibara_interfaces::msg::FaceEmbed>::SharedPtr face_subscription;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr spectogram_subscription;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_subscription;

    rclcpp::Subscription<kapibara_interfaces::msg::Emotions>::SharedPtr emotions_subscription;

    rclcpp::Service<kapibara_interfaces::srv::StopMind>::SharedPtr stop_mind_service;


    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;

    rclcpp::TimerBase::SharedPtr network_timer;


    long double last_reward;

    double yaw;


    float max_linear_speed;
    float max_angular_speed;

    float angular_p;
    float angular_i;

    double integral;

    uint64_t current_bag_id;

    bool check_for_point(point pt,uint64_t& id)
    {
        sqlite3_stmt *stmt;

        int rc = SQLITE_OK;

        rc = sqlite3_prepare_v2(this->db,
            "SELECT "
            "  id, "
            "  distance "
            "FROM vec_points "
            "WHERE position MATCH ?1 AND k = 3 "
            "ORDER BY distance "
            "LIMIT 1 "
        , -1, &stmt, NULL);

        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        float position[3] = {pt.x,pt.y,pt.w};

        sqlite3_bind_blob(stmt, 1, position, sizeof(position), SQLITE_STATIC);

        rc = sqlite3_step(stmt);

        sqlite3_int64 rowid = -1;

        if( rc == SQLITE_ROW )
        {
            rowid = sqlite3_column_int64(stmt, 0);
            double distance = sqlite3_column_double(stmt, 1);

            if( distance > 0.01 )
            {
                rowid = -1;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),"SQlite select error: %s",sqlite3_errmsg(this->db));
        }

        sqlite3_finalize(stmt);

        id = rowid;
        
        return rowid != -1;
    }

    // Add new point to databse if point at current coordinates exists, replace it
    // If no bag is selected create new one.
    void add_point_to_database(point pt)
    {
        // check if point aleardy exits

        uint64_t point_id = 0;

        if(check_for_point(pt,point_id))
        {
            RCLCPP_INFO(this->get_logger(),"Updating exisiting point at x: %i y: %i",pt.x,pt.y);
            this->update_point_in_database(pt,point_id);
            return;
        }


        // add point to database
        RCLCPP_INFO(this->get_logger(),"Adding new point at x: %i y: %i",pt.x,pt.y);

        sqlite3_stmt *stmt;
        
        int rc = SQLITE_OK;

        // add new point to database
        rc = sqlite3_exec(this->db, "BEGIN", NULL, NULL, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        rc = sqlite3_prepare_v2(this->db, "INSERT INTO vec_points(emotion_state,position) VALUES (?)", -1, &stmt, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        float position[3] = {pt.x,pt.y,pt.w};

        sqlite3_bind_double(stmt,1,pt.emotion_state);

        sqlite3_bind_blob(stmt,2,position,sizeof(position), SQLITE_STATIC);

        rc = sqlite3_step(stmt);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        rc = sqlite3_exec(this->db, "COMMIT", NULL, NULL, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        uint64_t inserted_point_id = sqlite3_last_insert_rowid(this->db);

        this->add_point_to_bag(pt,inserted_point_id);
    }

    // Create a new bag and get it's id and hold it to current_bag_id
    void create_new_bag()
    {
        // how to generate embedding for bag?
    }

    // add a point to a bag, if point is to far away from lastly added point new bag is created
    void add_point_to_bag(point pt,uint64_t point_id)
    {

        if( this->current_bag_id == 0 )
        {
            this->create_new_bag();
        }
        
        sqlite3_stmt *stmt;

        int rc = SQLITE_OK;

        // add new point to database
        rc = sqlite3_exec(this->db, "BEGIN", NULL, NULL, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        rc = sqlite3_prepare_v2(this->db, "INSERT INTO bag_point(emotion_state,bag_id,point_id) VALUES (?)", -1, &stmt, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        sqlite3_bind_int64(stmt,1,this->current_bag_id);
        sqlite3_bind_int64(stmt,2,point_id);

        rc = sqlite3_step(stmt);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        rc = sqlite3_exec(this->db, "END", NULL, NULL, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );
        
    }

    // update point in database, if point with id exists, update it
    void update_point_in_database(point pt,uint64_t id)
    {
        int rc = SQLITE_OK;
        sqlite3_stmt *stmt;

        rc = sqlite3_exec(this->db, "BEGIN", NULL, NULL, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );
        rc = sqlite3_prepare_v2(this->db, "UPDATE vec_points SET emotion_state = ? WHERE id = ?", -1, &stmt, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        sqlite3_bind_double(stmt,1,pt.emotion_state);
        sqlite3_bind_int64(stmt,2,id);

        rc = sqlite3_step(stmt);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        sqlite3_finalize(stmt);
        rc = sqlite3_exec(this->db, "COMMIT", NULL, NULL, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );
    }

    // takes current snapshot and use encoders or just database to retrive points to map
    // save current bag id
    void retrive_points()
    {

    }

    // add point to map, if point with coordinates exists, replace it
    void add_point_to_map(point pt)
    {
        // use quad tree to check if point exists in map
    }


    void stop_mind_handle(const std::shared_ptr<kapibara_interfaces::srv::StopMind::Request> request,
        std::shared_ptr<kapibara_interfaces::srv::StopMind::Response> response)
    {
        bool stop = request->stop;
        RCLCPP_INFO(this->get_logger(),"Stop mind request: %s",stop?"true":"false");

        if(stop)
        {
            this->stop_motors();
            this->network_timer->cancel();
        }
        else
        {
            this->network_timer->reset();
        }

        response->ok = true;
    }

    const std::string checkpoint_filename() const
    {
        return this->get_parameter("checkpoint_dir").as_string();
    }

    void orientation_callback(const sensor_msgs::msg::Imu::SharedPtr imu)
    {
        RCLCPP_DEBUG(this->get_logger(),"Got orientaion message!");

        // append orientaion data

        // this->inputs[0] = static_cast<number>(imu->orientation.x);
        // this->inputs[1] = static_cast<number>(imu->orientation.y);
        // this->inputs[2] = static_cast<number>(imu->orientation.z);
        // this->inputs[3] = static_cast<number>(imu->orientation.w);

    }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        RCLCPP_DEBUG(this->get_logger(),"Got odometry message!");

        // const auto& twist = odom->twist.twist;

        const auto& pose = odom->pose.pose;

        this->position[0] = static_cast<number>(pose.position.x)*STEP_SIZE;
        this->position[1] = static_cast<number>(pose.position.y)*STEP_SIZE;
        this->position[2] = static_cast<number>(pose.position.z)*STEP_SIZE;

        this->current_snapshot.x = this->position[0];
        this->current_snapshot.y = this->position[1];

        tf2::Quaternion quat;

        tf2::fromMsg(pose.orientation, quat);

        // raw, pitch, yaw
        double r,p,y;

        tf2::Matrix3x3 m(quat);

        m.getRPY(r,p,y);

        this->yaw = y;

        RCLCPP_DEBUG(this->get_logger(),"Robot coordinates: %f, %f,%f yaw: %f",
        pose.position.x,pose.position.y,pose.position.z,this->yaw);

        // append orientaion data

        // this->inputs[4] = static_cast<number>(twist.linear.x/MAX_LINEAR_SPEED);
        // this->inputs[5] = static_cast<number>(twist.linear.y/MAX_LINEAR_SPEED);
        // this->inputs[6] = static_cast<number>(twist.linear.z/MAX_LINEAR_SPEED);

        // this->inputs[7] = static_cast<number>(twist.angular.x/MAX_ANGULAR_SPEED);
        // this->inputs[8] = static_cast<number>(twist.angular.y/MAX_ANGULAR_SPEED);
        // this->inputs[9] = static_cast<number>(twist.angular.z/MAX_ANGULAR_SPEED);

    }

    void face_callback(const kapibara_interfaces::msg::FaceEmbed::SharedPtr face)
    {
        RCLCPP_DEBUG(this->get_logger(),"Got face embedding message!");

        // const std::vector<float>& embeddings = face->embedding;

        // size_t iter = 10;

        // float max_value = *std::max_element(embeddings.begin(), embeddings.end());

        // float min_value = *std::min_element(embeddings.begin(), embeddings.end());

        // for(const float elem : embeddings)
        // {

        //     this->inputs[iter] = static_cast<number>((elem - min_value)/(max_value - min_value));

        //     iter++;
        // }
    }

    void spectogram_callback(const sensor_msgs::msg::Image::SharedPtr img)
    {
        RCLCPP_DEBUG(this->get_logger(),"Got spectogram message!");

        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot convert image exception: %s", e.what());
            return;
        }

        cv::Mat& image = cv_ptr->image;

        cv::Mat spectogram;

        resize(image, spectogram, cv::Size(90, 90), cv::INTER_LINEAR); 

        size_t iter = 426;

        // for(size_t y=0;y<16;y++)
        // {
        //     for(size_t x=0;x<16;x++)
        //     {
        //         this->inputs[iter++] = static_cast<float>(spectogram.at<uint8_t>(x,y))/255.f;
        //     }
        // }

        for(size_t y=0;y<90;y++)
        {
            for(size_t x=0;x<90;x++)
            {
                this->current_snapshot.spectogram[y*90+x] = static_cast<float>(spectogram.at<uint8_t>(x,y))/255.f;
            }
        }

    }

    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr img)
    {
        RCLCPP_DEBUG(this->get_logger(),"Got image message!");

        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(img);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot convert image exception: %s", e.what());
            return;
        }

        cv::Mat& image = cv_ptr->image;

        cv::Mat gray;

        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        cv::Mat new_image;

        resize(gray, new_image, cv::Size(90, 90), cv::INTER_CUBIC); 

        cv::Mat laplce_img;

        cv::Laplacian(new_image,laplce_img,CV_64F);

        for(size_t y=0;y<90;y++)
        {
            for(size_t x=0;x<90;x++)
            {
                this->current_snapshot.image[y*90+x] = static_cast<float>(new_image.at<uint8_t>(x,y))/255.f;
            }
        }

        for(size_t y=0;y<90;y++)
        {
            for(size_t x=0;x<90;x++)
            {
                this->current_snapshot.image_laplace[y*90+x] = laplce_img.at<float>(x,y)/255.f;
            }
        }
    }

    void emotions_callback(const kapibara_interfaces::msg::Emotions::SharedPtr emotions)
    {
        RCLCPP_INFO(this->get_logger(),"Got emotions message!");

        float reward = emotions->happiness*320.f + emotions->fear*-120.f + emotions->uncertainty*-40.f + emotions->angry*-60.f + emotions->boredom*-20.f;

        this->last_reward = reward;

        // propagate rewards to aleardy visited fields
        float _reward = reward;

        for(size_t i=0;i<this->snapshots.length();++i)
        {
            int32_t x = this->snapshots.get(i).x;
            int32_t y = this->snapshots.get(i).y;

            if( abs(reward) > 0.1f )
            {
                // add point to map
                
            }

            // maybe let's use sqlite database to stroe points

            _reward*=0.95f;
        }
    }


    void network_callback()
    {
        RCLCPP_INFO(this->get_logger(),"Network fired!");

        const double time_stamp = 0.1f;

        // retrive bags based on current sensory input
        

        // get current position
        double x = this->current_snapshot.x;
        double y = this->current_snapshot.y;

        // linear forces

        double x_force = 0.f;
        double y_force = 0.f;

        // iterate over all points in the map
        for(size_t i=0;i<this->map_points.size();++i)
        {
            const point& pt = this->map_points[i];

            // calculate distance to point
            double dx = static_cast<double>(pt.x) - x;
            double dy = static_cast<double>(pt.y) - y;

            double distance2 = dx*dx + dy*dy;

            double force = LINEAR_FORCE_COF / (distance2 + 1e-6);
            
            double angle = std::atan2(dy, dx);

            x_force += force * std::cos(angle);
            y_force += force * std::sin(angle);
        }

        double target_angle = std::atan2(y_force, x_force);

        double angle_error = target_angle - this->yaw;

        if( abs(angle_error) > 0.1)
        {
            this->integral += angle_error*time_stamp;

            this->integral = std::clamp<double>(this->integral, -this->max_angular_speed, this->max_angular_speed);

            double angular_speed = this->angular_p * angle_error + this->angular_i * this->integral;

            this->send_twist(0.f, std::clamp<double>(angular_speed, -this->max_angular_speed, this->max_angular_speed));

            return;
        }

        // use 10 Hz

        this->integral = 0.f;

        double force = x_force*x_force + y_force*y_force;

        this->send_twist(
            std::clamp<double>(force*0.125f, 0.f, this->max_linear_speed),
            0.f
        );

    }

    // get id of field associated to current image, return -1 if not found
    void get_field_id_from_database(sqlite3_int64 ids[], const snapshot& snap)
    {
        sqlite3_stmt *stmt;

        int rc = SQLITE_OK;

        rc = sqlite3_prepare_v2(this->db,
            "SELECT "
            "  id, "
            "  distance "
            "FROM vec_images "
            "WHERE images_embedding MATCH ?1 AND k = 3 "
            "ORDER BY distance "
            "LIMIT 1 "
        , -1, &stmt, NULL);

        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        sqlite3_bind_blob(stmt, 1, snap.image, sizeof(snap.image), SQLITE_STATIC);

        rc = sqlite3_step(stmt);

        sqlite3_int64 rowid = -1;

        if( rc == SQLITE_ROW )
        {
            rowid = sqlite3_column_int64(stmt, 0);
            double distance = sqlite3_column_double(stmt, 1);

            if( distance > 0.01 )
            {
                rowid = -1;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),"SQlite select error: %s",sqlite3_errmsg(this->db));
        }

        ids[0] = rowid;

        sqlite3_finalize(stmt);

        // now select by laplace image embedding

        rc = sqlite3_prepare_v2(this->db,
            "SELECT "
            "  id, "
            "  distance "
            "FROM vec_images_laplace "
            "WHERE images_embedding MATCH ?1 AND k = 3 "
            "ORDER BY distance "
            "LIMIT 1 "
        , -1, &stmt, NULL);

        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        sqlite3_bind_blob(stmt, 1, snap.image_laplace, sizeof(snap.image_laplace), SQLITE_STATIC);

        rc = sqlite3_step(stmt);

        rowid = -1;

        if( rc == SQLITE_ROW )
        {
            rowid = sqlite3_column_int64(stmt, 0);
            double distance = sqlite3_column_double(stmt, 1);

            if( distance > 0.01 )
            {
                rowid = -1;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),"SQlite select error: %s",sqlite3_errmsg(this->db));
        }

        ids[1] = rowid;

        sqlite3_finalize(stmt);


        // now select by spectogram embedding

        rc = sqlite3_prepare_v2(this->db,
            "SELECT "
            "  id, "
            "  distance "
            "FROM vec_spectogram "
            "WHERE images_embedding MATCH ?1 AND k = 3 "
            "ORDER BY distance "
            "LIMIT 1 "
        , -1, &stmt, NULL);

        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        sqlite3_bind_blob(stmt, 1, snap.spectogram, sizeof(snap.spectogram), SQLITE_STATIC);

        rc = sqlite3_step(stmt);

        rowid = -1;

        if( rc == SQLITE_ROW )
        {
            rowid = sqlite3_column_int64(stmt, 0);
            double distance = sqlite3_column_double(stmt, 1);

            if( distance > 0.01 )
            {
                rowid = -1;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),"SQlite select error: %s",sqlite3_errmsg(this->db));
        }

        ids[2] = rowid;

        sqlite3_finalize(stmt);

    }

    void get_field_by_id(sqlite3_int64 id, float* field)
    {
        RCLCPP_INFO(this->get_logger(),"Got field id: %lli",id);

        if( id <= -1 )
        {
            for(size_t i=0;i<8*8;++i)
            {
                field[i] = 0.f;
            }

            return;
        }


        sqlite3_stmt *stmt;

        int rc = SQLITE_OK;

        rc = sqlite3_prepare_v2(this->db,
            "SELECT "
            "  field "
            "FROM images "
            "WHERE id = ? "
            "LIMIT 1"
        , -1, &stmt, NULL);

        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        sqlite3_bind_int64(stmt,1,id);

        rc = sqlite3_step(stmt);

        float *_field;

        if( rc == SQLITE_ROW )
        {
            _field = (float*)sqlite3_column_blob(stmt,0);

            for(size_t i=0;i<8*8;++i)
            {
                field[i] = _field[i];
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),"SQlite select error: %s",sqlite3_errmsg(this->db));
        }

        sqlite3_finalize(stmt);

    }

    void update_field_with_id(sqlite3_int64 id)
    {
        this->update_field_with_id(id,this->current_snapshot);
    }

    void update_field_with_id(sqlite3_int64 id,const snapshot& snap)
    {
        int rc = SQLITE_OK;
        sqlite3_stmt *stmt;

        rc = sqlite3_exec(this->db, "BEGIN", NULL, NULL, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );
        rc = sqlite3_prepare_v2(this->db, "UPDATE images SET field = ? WHERE id = ?", -1, &stmt, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        float field[8*8] = {0.f};

        // to do
        // this->get_current_field(field,snap);s

        sqlite3_bind_blob(stmt,1,field,sizeof(field), SQLITE_STATIC);
        sqlite3_bind_int64(stmt,2,id);

        rc = sqlite3_step(stmt);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        sqlite3_finalize(stmt);
        rc = sqlite3_exec(this->db, "COMMIT", NULL, NULL, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );
    }

    void update_database()
    {
        this->update_database(this->current_snapshot);
    }

    // update database with current image embedding and positions 
    void update_database(const snapshot& snap)
    {
        // we should check if image is aleardy present in database

        sqlite3_int64 id[3];

        this->get_field_id_from_database(id,snap);

        RCLCPP_INFO(this->get_logger(),"Got id up database: %lli,%lli,%lli",id[0],id[1],id[2]);

        if( id[0] > -1 )
        {
            // update exisiting field
            this->update_field_with_id(id[0],snap);
            return;
        }

        if( id[1] > -1 )
        {
            // update exisiting field
            this->update_field_with_id(id[1],snap);
            return;
        }

        if( id[2] > -1 )
        {
            // update exisiting field
            this->update_field_with_id(id[2],snap);
            return;
        }

        sqlite3_stmt *stmt;
        
        int rc = SQLITE_OK;

        // add new image embedding
        rc = sqlite3_exec(this->db, "BEGIN", NULL, NULL, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );
        rc = sqlite3_prepare_v2(this->db, "INSERT INTO vec_images(images_embedding) VALUES (?)", -1, &stmt, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        sqlite3_bind_blob(stmt,1,snap.image,sizeof(snap.image), SQLITE_STATIC);
        rc = sqlite3_step(stmt);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        sqlite3_finalize(stmt);
        rc = sqlite3_exec(this->db, "COMMIT", NULL, NULL, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        // add new image laplace embedding
        rc = sqlite3_exec(this->db, "BEGIN", NULL, NULL, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );
        rc = sqlite3_prepare_v2(this->db, "INSERT INTO vec_images_laplace(images_embedding) VALUES (?)", -1, &stmt, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        sqlite3_bind_blob(stmt,1,snap.image_laplace,sizeof(snap.image_laplace), SQLITE_STATIC);
        rc = sqlite3_step(stmt);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        sqlite3_finalize(stmt);
        rc = sqlite3_exec(this->db, "COMMIT", NULL, NULL, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        // add new image spectogram embedding
        rc = sqlite3_exec(this->db, "BEGIN", NULL, NULL, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );
        rc = sqlite3_prepare_v2(this->db, "INSERT INTO vec_spectogram(images_embedding) VALUES (?)", -1, &stmt, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        sqlite3_bind_blob(stmt,1,snap.spectogram,sizeof(snap.spectogram), SQLITE_STATIC);
        rc = sqlite3_step(stmt);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        sqlite3_finalize(stmt);
        rc = sqlite3_exec(this->db, "COMMIT", NULL, NULL, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        // get field 

        float field[8*8] = {0.f};

        // to do
        // this->get_current_field(field,snap);


        rc = sqlite3_exec(this->db, "BEGIN", NULL, NULL, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );
        rc = sqlite3_prepare_v2(this->db, "INSERT INTO images(field) VALUES (?)", -1, &stmt, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        sqlite3_bind_blob(stmt,1,field,sizeof(field), SQLITE_STATIC);
        rc = sqlite3_step(stmt);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        sqlite3_finalize(stmt);
        rc = sqlite3_exec(this->db, "COMMIT", NULL, NULL, NULL);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        
    }

    public:

    KapiBaraMind()
    : Node("kapibara_mind")
    {
        this->declare_parameter("checkpoint_dir", "/app/src/mind.kac");

        this->declare_parameter("max_linear_speed", 2.f);

        this->declare_parameter("max_angular_speed", 3.f);

        this->declare_parameter("angular_p", 4.f);

        this->declare_parameter("angular_i", 2.f);

        this->current_bag_id = 0;

        this->last_reward = 0.f;

        this->integral = 0.0;

        this->init_database();

        this->max_linear_speed = this->get_parameter("max_linear_speed").as_double();
        this->max_angular_speed = this->get_parameter("max_angular_speed").as_double();

        this->angular_p = this->get_parameter("angular_p").as_double();
        this->angular_i = this->get_parameter("angular_i").as_double();;

        // add all required subscriptions

        this->orientation_subscription = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&KapiBaraMind::orientation_callback, this, _1));

        this->odometry_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
      "motors/odom", 10, std::bind(&KapiBaraMind::odometry_callback, this, _1));

        this->face_subscription = this->create_subscription<kapibara_interfaces::msg::FaceEmbed>(
      "spoted_faces", 10, std::bind(&KapiBaraMind::face_callback, this, _1));

        this->spectogram_subscription = this->create_subscription<sensor_msgs::msg::Image>(
      "spectogram", 10, std::bind(&KapiBaraMind::spectogram_callback, this, _1));

      this->image_subscription = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "camera/image_raw/compressed", 10, std::bind(&KapiBaraMind::image_callback, this, _1));

        this->emotions_subscription = this->create_subscription<kapibara_interfaces::msg::Emotions>(
      "emotions", 10, std::bind(&KapiBaraMind::emotions_callback, this, _1));

        this->stop_mind_service = this->create_service<kapibara_interfaces::srv::StopMind>(
      "stop_mind", std::bind(&KapiBaraMind::stop_mind_handle, this, std::placeholders::_1, std::placeholders::_2));


        // one publisher for ros2 control cmd

        this->twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>("motors/cmd_vel_unstamped", 10);

        this->network_timer = this->create_wall_timer(100ms, std::bind(&KapiBaraMind::network_callback, this));
        
        this->field_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("field", 10);
        
    }

    void send_twist(float linear,float angular)
    {
        geometry_msgs::msg::Twist twist;

        twist.angular.z = angular;

        twist.linear.x = linear;

        this->twist_publisher->publish(twist);
    }


    void stop_motors()
    {
        this->send_twist(0.f,0.f);
    }


    void shutdown()
    {
        this->stop_motors();
    }

    inline void validate_sqlite(int rc)
    {
        if( rc != SQLITE_OK && rc != SQLITE_DONE )
        {
            RCLCPP_ERROR(this->get_logger(),"SQLITE error: %s",sqlite3_errmsg(this->db));
        }
    }

    void init_database()
    {
        int rc = SQLITE_OK;

        // create database
        rc = sqlite3_open("mind_database.db", &db);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );

        sqlite3_stmt *stmt;

        // create vector table if it doesn't exist

        RCLCPP_INFO(this->get_logger(),"Creating database tables...");

        RCLCPP_INFO(this->get_logger(),"Creating vec_images database...");

        rc = sqlite3_prepare_v2(db, "CREATE VIRTUAL TABLE IF NOT EXISTS vec_images USING vec0(id INTEGER PRIMARY KEY, images_embedding FLOAT[8100])", -1, &stmt, NULL);
        
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );
        rc = sqlite3_step(stmt);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );
        sqlite3_finalize(stmt);


        // create vector table if it doesn't exist ( laplace image )

        RCLCPP_INFO(this->get_logger(),"Creating vec_images_laplace database...");

        rc = sqlite3_prepare_v2(db, "CREATE VIRTUAL TABLE IF NOT EXISTS vec_images_laplace USING vec0(id INTEGER PRIMARY KEY, images_embedding FLOAT[8100])", -1, &stmt, NULL);
        
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );
        rc = sqlite3_step(stmt);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );
        sqlite3_finalize(stmt);

        // create vector table if it doesn't exist ( spectograms )
        RCLCPP_INFO(this->get_logger(),"Creating spectogram database...");

        rc = sqlite3_prepare_v2(db, "CREATE VIRTUAL TABLE IF NOT EXISTS vec_spectograms USING vec0(id INTEGER PRIMARY KEY, images_embedding FLOAT[8100])", -1, &stmt, NULL);
        
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );
        rc = sqlite3_step(stmt);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );
        sqlite3_finalize(stmt);

        // create table for points
        RCLCPP_INFO(this->get_logger(),"Creating point database...");

        rc = sqlite3_prepare_v2(db, "CREATE TABLE IF NOT EXISTS vec_points(id INTEGER PRIMARY KEY,emotion_state FLOAT,position FLOAT[3])", -1, &stmt, NULL);

        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );
        rc = sqlite3_step(stmt);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );
        sqlite3_finalize(stmt);

        // create table for bags
        RCLCPP_INFO(this->get_logger(),"Creating bags database...");

        rc = sqlite3_prepare_v2(db, "CREATE TABLE IF NOT EXISTS vec_bag(id INTEGER PRIMARY KEY,embeddings FLOAT[64])", -1, &stmt, NULL);

        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );
        rc = sqlite3_step(stmt);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );
        sqlite3_finalize(stmt); 

        // create table for holding pair of bag and point
        RCLCPP_INFO(this->get_logger(),"Creating bags and point relation database...");

        rc = sqlite3_prepare_v2(db, "CREATE TABLE IF NOT EXISTS bag_point(id INTEGER PRIMARY KEY,bag_id INTEGER,point_id INTEGER)", -1, &stmt, NULL);

        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );
        rc = sqlite3_step(stmt);
        this->validate_sqlite(rc);
        assert( rc == SQLITE_OK || rc == SQLITE_DONE );
        sqlite3_finalize(stmt); 

    }

    void close_database()
    {
        sqlite3_close(db);
    }

    ~KapiBaraMind()
    {
        this->close_database();
    }   

};


std::function<void(int)> shutdown_handler;
void signal_handler(int signal) { shutdown_handler(signal); }


int main(int argc,char** argv)
{   
    rclcpp::init(argc, argv);

    // init sqlite vec
    int rc = SQLITE_OK;

    rc = sqlite3_auto_extension((void (*)())sqlite3_vec_init);
    assert(rc == SQLITE_OK);

    auto node = std::make_shared<KapiBaraMind>();

    shutdown_handler = [node](int signal)->void    {
        
        rclcpp::shutdown();

        node->shutdown();

        exit(0);

    };

    std::signal(SIGINT,signal_handler);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;

}

