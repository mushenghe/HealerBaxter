from __future__ import print_function
import keras
from keras.preprocessing.image import ImageDataGenerator
from keras.models import Sequential
from keras.layers import Dense,Dropout,Activation,Flatten,BatchNormalization
from keras.layers import Conv2D,MaxPooling2D
import os

num_classes = 5 # Angry, Happy, Neutral, Sad, Surprise
img_rows,img_cols = 48,48
batch_size = 64 # use 32 images one time for training,can change

train_data_dir = '/home/mushenghe/emotion_detection/database/fer2013/train/'
validation_data_dir = '/home/mushenghe/emotion_detection/database/fer2013/validation/'

train_datagen = ImageDataGenerator(
					rescale=1./255,    #convert to grayscale
					rotation_range=30,   #generate many kinds of images by rotate 30 degree to the left and to the right
					shear_range=0.3, 
					zoom_range=0.3,  # zoom in and out
					width_shift_range=0.4, #shift
					height_shift_range=0.4,
					horizontal_flip=True, #flip
					fill_mode='nearest')

validation_datagen = ImageDataGenerator(rescale=1./255) #normalize

train_generator = train_datagen.flow_from_directory(
					train_data_dir,
					color_mode='grayscale',
					target_size=(img_rows,img_cols),
					batch_size=batch_size,
					class_mode='categorical',
					shuffle=True)

validation_generator = validation_datagen.flow_from_directory(
							validation_data_dir,
							color_mode='grayscale',
							target_size=(img_rows,img_cols),
							batch_size=batch_size,
							class_mode='categorical',
							shuffle=True)

# define convolutional neural net
model = Sequential()

# Block-1
## apply 64 convolution filters of size 3x3 each.
model.add(Conv2D(64,(3,3),padding='same',kernel_initializer='he_normal',input_shape=(img_rows,img_cols,1))) #48*48 grayscale image
model.add(Activation('relu'))
model.add(BatchNormalization())
model.add(Conv2D(64,(3,3),padding='same',kernel_initializer='he_normal',input_shape=(img_rows,img_cols,1)))
model.add(Activation('relu'))
model.add(BatchNormalization())
model.add(MaxPooling2D(pool_size=(2,2)))
model.add(Dropout(0.25)) # at a time only 75% neurons will activated, prevent the model from overfitting

# Block-2 

model.add(Conv2D(128,(5,5),padding='same',kernel_initializer='he_normal'))
model.add(Activation('relu'))
model.add(BatchNormalization())
model.add(Conv2D(128,(5,5),padding='same',kernel_initializer='he_normal'))
model.add(Activation('relu'))
model.add(BatchNormalization())
model.add(MaxPooling2D(pool_size=(2,2)))
model.add(Dropout(0.25))

# Block-3

model.add(Conv2D(128,(3,3),padding='same',kernel_initializer='he_normal'))
model.add(Activation('relu'))
model.add(BatchNormalization())
model.add(Conv2D(128,(3,3),padding='same',kernel_initializer='he_normal'))
model.add(Activation('relu'))
model.add(BatchNormalization())
model.add(MaxPooling2D(pool_size=(2,2)))
model.add(Dropout(0.25))

# Block-4 

model.add(Conv2D(256,(5,5),padding='same',kernel_initializer='he_normal'))
model.add(Activation('elu'))
model.add(BatchNormalization())
model.add(Conv2D(256,(5,5),padding='same',kernel_initializer='he_normal'))
model.add(Activation('elu'))
model.add(BatchNormalization())
model.add(MaxPooling2D(pool_size=(2,2)))
model.add(Dropout(0.25))

# Block-5 

model.add(Conv2D(512,(3,3),padding='same',kernel_initializer='he_normal'))
model.add(Activation('relu'))
model.add(BatchNormalization())
model.add(Conv2D(512,(3,3),padding='same',kernel_initializer='he_normal'))
model.add(Activation('relu'))
model.add(BatchNormalization())
model.add(MaxPooling2D(pool_size=(2,2)))
model.add(Dropout(0.25))


# start flatten process
# use flatten layer
# Block-5

model.add(Flatten())
model.add(Dense(512,kernel_initializer='he_normal'))
model.add(Activation('relu'))
model.add(BatchNormalization())
model.add(Dropout(0.25))

# Block-6

model.add(Dense(1024,kernel_initializer='he_normal'))
model.add(Activation('relu'))
model.add(BatchNormalization())
model.add(Dropout(0.25))

# Block-7

model.add(Dense(num_classes,kernel_initializer='he_normal'))
model.add(Activation('softmax'))

print(model.summary())

from keras.optimizers import RMSprop,SGD,Adam
from keras.callbacks import ModelCheckpoint, EarlyStopping, ReduceLROnPlateau

checkpoint = ModelCheckpoint('little_vgg.h5',
                              monitor='val_loss', #monitor the validation lost
                              mode='min',
                              save_best_only=True, # only save the best result
                             verbose=1)

earlystop = EarlyStopping(monitor='val_loss',
                           min_delta=0,
                           patience=20,
                           verbose=1,
                           restore_best_weights=True
                           )

reduce_lr = ReduceLROnPlateau(monitor='val_loss',
                               factor=0.2,
                               patience=3,
                               verbose=1,
                               min_delta=0.0001)

callbacks = [earlystop,checkpoint,reduce_lr]

model.compile(loss='categorical_crossentropy',
               optimizer = Adam(lr=0.001),
               metrics=['accuracy'])

nb_train_samples = 7753413
nb_validation_samples = 7424
epochs=120

history=model.fit_generator(
                train_generator,
                steps_per_epoch=nb_train_samples//batch_size,
                epochs=epochs,
                callbacks=callbacks,
                validation_data=validation_generator,
                validation_steps=nb_validation_samples//batch_size)
