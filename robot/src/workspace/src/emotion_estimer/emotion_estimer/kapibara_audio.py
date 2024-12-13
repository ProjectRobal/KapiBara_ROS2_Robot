import numpy as np
import tensorflow as tf

import platform

from tensorflow_model_optimization.python.core.keras.compat import keras

# import keras

# from keras import layers
# from keras import models
import tensorflow_model_optimization as tfmot

try:
    from tflite_runtime.interpreter import Interpreter
    from tflite_runtime.interpreter import load_delegate
except:
    from tensorflow.lite.python.interpreter import Interpreter
    from tensorflow.lite.python.interpreter import load_delegate
    

EDGETPU_SHARED_LIB = {
  'Linux': 'libedgetpu.so.1',
  'Darwin': 'libedgetpu.1.dylib',
  'Windows': 'edgetpu.dll'
}[platform.system()]

BUFFER_SIZE = 16000*2

OUTPUTS=5


class KapibaraAudio:
    '''path - a path to a model'''
    def __init__(self,path=None,tflite=False):
        self.model=None
        if path is not None:
            if not tflite:
                self.model=tf.keras.models.load_model(path)
            else:
                self.model = Interpreter( model_path=path,
                                         experimental_delegates=[load_delegate(EDGETPU_SHARED_LIB)])
                self.model.allocate_tensors()
                self.input_details = self.model.get_input_details()
                self.output_details = self.model.get_output_details()
            
        self.tflite = tflite

        self.answers=['neutral','unsettling','pleasent','scary','nervous']
        self.sample_rate=16000
        self.buffer_size=BUFFER_SIZE

    '''read samples from dataset'''
    def read_samples(self,dir,file="train.csv",delimiter=';'):
    
        audio=[]

        neutral=[]


        with open(dir+"/"+file,"r") as f:
            headers=f.readline()
            for line in f:
                objs=line.split(delimiter)

                for i in range(1,len(objs)):
                    objs[i]=objs[i].replace(',','.')
                    objs[i]=float(objs[i])

                audio.append(objs[0])
                neutral.append(tf.argmax(objs[1:]))
                

        
        return (audio,neutral)



    '''path - a path to dataset'''
    def train(self,path,batch_size=32,EPOCHS = 300,file="train.csv",valid="valid.csv",delimiter=";",save_path="./best_model.h5"):
        
        files,labels = self.read_samples(path,file,delimiter)

        spectrograms=[]
        

        for file in files:
            audio=self.load_wav(path+"/wavs/"+file+".wav")

            spectrograms.append(self.gen_spectogram(audio))

        print("Samples count: ",len(spectrograms))

        dataset=tf.data.Dataset.from_tensor_slices((spectrograms,labels))

        train_ds=dataset

        train_ds=train_ds.batch(batch_size)

        train_ds = train_ds.cache().prefetch(tf.data.AUTOTUNE)

        #validation dataset

        files,labels = self.read_samples(path,valid,delimiter)

        spectrograms.clear()

        for file in files:
            audio=self.load_wav(path+"/wavs/"+file+".wav")

            spectrograms.append(self.gen_spectogram(audio))

        valid_ds=tf.data.Dataset.from_tensor_slices((spectrograms,labels))

        valid_ds=valid_ds.batch(batch_size)

        valid_ds=valid_ds.cache().prefetch(tf.data.AUTOTUNE)

        for spectrogram, _ in dataset.take(1):
            input_shape = spectrogram.shape

        #a root 
        # a input of a size 64 x 64
        
        model = keras.Sequential()
        model.add(keras.Input(shape=input_shape))

        # Instantiate the `tf.keras.layers.Normalization` layer.
        # norm_layer = keras.layers.Normalization()
        # Fit the state of the layer to the spectrograms
        # with `Normalization.adapt`.
        # norm_layer.adapt(data=dataset.map(map_func=lambda spec, label: spec))
        
        # norm_layer(input_layer)
        
        # model.add(norm_layer)

        model.add(keras.layers.Conv2D(8, 3, activation='relu'))

        # conv2=layers.Conv2D(16, 3, activation='relu')(conv1)

        model.add(keras.layers.Flatten())

        #output layers

        model.add(keras.layers.Dense(16, activation='relu'))
        
        model.add(keras.layers.Dense(24, activation='relu'))
        
        model.add(keras.layers.Dense(24, activation='relu'))
                
        model.add(keras.layers.Dense(24, activation='relu'))

        model.add(keras.layers.Dense(OUTPUTS,activation='softmax'))


        # model=models.Model(inputs=input_layer,outputs=neutral_output)
        
        quantize_model = tfmot.quantization.keras.quantize_model
        
        # q_aware stands for for quantization aware
        q_aware_model = quantize_model(model)

        q_aware_model.summary()

        q_aware_model.compile(
            optimizer=keras.optimizers.Adam(),
            loss=keras.losses.SparseCategoricalCrossentropy(from_logits=True),
            metrics=['accuracy']
        )

        
        history = q_aware_model.fit(
            train_ds,
            validation_data=valid_ds,
            epochs=EPOCHS
            )
        
        q_aware_model.save(save_path)
        
        def representative_dataset_gen():
            for data in valid_ds:
                # data = tf.cast(data[0], dtype=tf.int8)
                yield [data[0]]
                
        # model.load_weights("best_model.h5")
        
        converter = tf.lite.TFLiteConverter.from_keras_model(q_aware_model)
        converter.optimizations = [tf.lite.Optimize.DEFAULT]
        converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
        converter.representative_dataset = representative_dataset_gen
        converter.inference_input_type = tf.float32
        converter.inference_output_type = tf.float32

        quantized_tflite_model = converter.convert()
        
        # quantized_tflite_model.save()        
        
        with open('model.tflite',"wb") as file:
            file.write(quantized_tflite_model)
            

        return history


    '''generate spectogram'''
    def gen_spectogram(self,audio):
    
        spectrogram=tf.signal.stft(audio,frame_length=255,frame_step=128)

        spectrogram = tf.abs(spectrogram)
        
        
        spectrogram = tf.image.resize(spectrogram[..., tf.newaxis],[64,64])

        return spectrogram

    def get_result(self,prediction):

        return self.answers[tf.argmax(prediction.numpy()[0])]

    '''audio - raw audio input'''
    def input(self,audio):

        if audio.shape[0]<BUFFER_SIZE:
            zeros=tf.zeros(BUFFER_SIZE-audio.shape[0])
            audio=tf.concat([audio,zeros],0)

        if audio.shape[0]>BUFFER_SIZE:
            audio=tf.slice(audio,0,BUFFER_SIZE)

        spectrogram=self.gen_spectogram(audio)[None,...,tf.newaxis]
        
        
        if self.tflite:
            self.model.set_tensor(self.input_details[0]['index'],spectrogram)
            self.model.invoke()
            
            prediction = self.model.get_tensor(self.output_details[0]['index'])
            
            return self.get_result(prediction)

        prediction = self.model(spectrogram)

        return self.get_result(prediction)

    '''path - a path to the wav file'''
    def load_wav(self,path):
        audio, _ = tf.audio.decode_wav(contents=tf.io.read_file(path))

        audio=tf.squeeze(audio, axis=-1)

        if audio.shape[0]<BUFFER_SIZE:
            zeros=tf.zeros(BUFFER_SIZE-audio.shape[0])
            audio=tf.concat([audio,zeros],0)

        if audio.shape[0]>BUFFER_SIZE:
            audio=tf.slice(audio,[0],[BUFFER_SIZE])

        audio=tf.cast(audio,dtype=tf.float32)

        return audio

    '''path - a path to the wav file'''
    def input_wav(self, path):

        audio=self.load_wav(path)

        return self.input(audio)
        


