import sqlite3
import time
import logging

from timeit import default_timer as timer


class FaceObj:
    def __init__(self,id:int,score:float,time:int):
        self.id = id
        self.score = score
        self.time = time

class FaceData:
    
    def _create_table(self):
        # string id, score , creation_time
        
        cursor = self.db.cursor()
        
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='FACE';")
        
        result = cursor.fetchall()
        
        if len(result)==0:
            table_query = '''
                CREATE TABLE FACE (
                ID INT NOT NULL PRIMARY KEY,
                SCORE FLOAT(4) NOT NULL,
                CREATION_TIME INT(8) NOT NULL
            );'''
            
            cursor.execute(table_query)
            
            cursor.fetchall()
                        
            self.logger.info("Created table FACE!")
            
        cursor.close()
            
    def __init__(self,database_path:str,log_level=logging.INFO):
        
        logging.basicConfig(level=log_level,
                    format='%(asctime)s - %(levelname)s - %(message)s')

        # Creating an object
        self.logger = logging.getLogger("face_data")
        
        self.logger.info("Connecting to database: "+str(database_path))
        # connect to database
        self.db = sqlite3.connect(database_path, check_same_thread=False)
        
        self.logger.info("Connected to database: "+str(database_path))
                
        # create table if it doesn't exist
        self._create_table()
    
    def add_face(self,id:int,score:float):
        
        try:
            
            self.logger.info("Adding face with id: "+str(id)+" and score: "+str(score))
            
            cursor = self.db.cursor()
            
            current_datetime = int(time.time())
            
            query = "INSERT INTO FACE VALUES ("+str(id)+","+str(score)+","+str(current_datetime)+");"
            
            self.logger.debug(query)
                    
            cursor.execute(query)
                
            result = cursor.fetchall()
            
            cursor.close()
        except sqlite3.Error as error:
            self.logger.error("Error occured when adding face: "+str(error))
        
    def get_face(self,id:int)->FaceObj|None:
        
        self.logger.info("Get face with id: "+str(id))
        
        cursor = self.db.cursor()
        
        query = "SELECT * FROM FACE WHERE ID = "+str(id)+";"
        
        self.logger.debug(query)
                
        cursor.execute(query)
            
        result = cursor.fetchall()
        
        cursor.close()
        
        if len(result)==0:
            return None
        
        return FaceObj(result[0][0],result[0][1],result[0][2])
    
    def get_oldest_entry(self)->FaceObj|None:
        
        self.logger.info("Get face with id: "+str(id))
        
        cursor = self.db.cursor()
        
        query = "SELECT * FROM FACE ORDER BY CREATION_TIME ASC LIMIT 1;"
        
        self.logger.debug(query)
                
        cursor.execute(query)
            
        result = cursor.fetchall()
        
        cursor.close()
        
        if len(result)==0:
            return None
        
        return FaceObj(result[0][0],result[0][1],result[0][2])
    
    def update_face_time(self,id:int):
        
        try:
            
            self.logger.info("Updating time of face with id: "+str(id))
            
            cursor = self.db.cursor()
            
            current_datetime = int(time.time())
            
            query = "UPDATE FACE SET CREATION_TIME="+str(current_datetime)+" WHERE ID = "+str(id)+";"
            
            self.logger.debug(query)
                    
            cursor.execute(query)
                
            cursor.fetchall()
            
            cursor.close()
        except sqlite3.Error as error:
            self.logger.error("Error occured when updating face: "+str(error))
    
    def update_face(self,id:int,score:float):
        try:
            
            self.logger.info("Updating face with id: "+str(id))
            
            cursor = self.db.cursor()
            
            current_datetime = int(time.time())
            
            query = "UPDATE FACE SET CREATION_TIME="+str(current_datetime)+",SCORE="+str(score)+" WHERE ID = "+str(id)+";"
            
            self.logger.debug(query)
                    
            cursor.execute(query)
                
            cursor.fetchall()
            
            cursor.close()
        except sqlite3.Error as error:
            self.logger.error("Error occured when updating face: "+str(error))
            
    def remove_face(self,id:int):
        try:
            
            self.logger.info("Removing face with id: "+str(id))
            
            cursor = self.db.cursor()
                        
            query = "DELETE FROM FACE WHERE ID = "+str(id)+";"
            
            self.logger.debug(query)
                    
            cursor.execute(query)
                
            cursor.fetchall()
            
            cursor.close()
        except sqlite3.Error as error:
            self.logger.error("Error occured when removing face: "+str(error))
            
    def commit(self):
        self.db.commit()
        
    def __del__(self):
        self.commit()
        
        
        
if __name__ == "__main__":
    
    db = FaceData("./db.sqlite")
    
    start = timer()
    
    for i in range(500):
        db.add_face(i,10.0)
    
    print("Time: "+str(timer()-start)+" s")
    
    print(db.get_face(11))
    
    db.update_face_time(11)
    
    print(db.get_face(11))
    
    db.update_face(27,-10)
    
    input()