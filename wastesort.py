import PIL
import os
import numpy as np
from tensorflow.keras.preprocessing import image
import numpy as np
import cv2


categories = ['Greenleaves', 'bottle', 'clothes', 'vegetables']
dataset_dir = r'C:\Users\jeevith\Downloads\python\dataset'

data = []
labels = []

for category in categories:
    path = os.path.join(dataset_dir, category)
    label = categories.index(category)
    print("-----------" + dataset_dir + "-----------" )
    for img in os.listdir(path):
        img_path = os.path.join(path, img)
        path = path.replace("\\","/")
        print("-----------" + img_path + "-----------" )
        img = image.load_img(img_path, target_size=(224, 224))
        img_array = image.img_to_array(img)
        img_array /= 255.0  # Normalize pixel values
        data.append(img_array)
        labels.append(label)

# Convert lists to NumPy arrays
data = np.array(data)
labels = np.array(labels)


from sklearn.model_selection import train_test_split

X_train, X_test, y_train, y_test = train_test_split(data, labels, test_size=0.1, random_state=42)


from tensorflow import keras
from tensorflow.keras import layers

model = keras.Sequential([
    layers.Conv2D(32, (3, 3), activation='relu', input_shape=(224, 224, 3)),
    layers.MaxPooling2D((2, 2)),
    layers.Conv2D(64, (3, 3), activation='relu'),
    layers.MaxPooling2D((2, 2)),
    layers.Conv2D(128, (3, 3), activation='relu'),
    layers.MaxPooling2D((2, 2)),
    layers.Flatten(),
    layers.Dense(128, activation='relu'),
    layers.Dense(len(categories), activation='softmax')
])

model.compile(optimizer='adam', loss='sparse_categorical_crossentropy', metrics=['accuracy'])


model.fit(X_train, y_train, epochs=12, validation_data=(X_test, y_test))


model.save('image_classifier_model.h5')

loss, accuracy = model.evaluate(X_test, y_test)
print(f'Test loss: {loss}')
print(f'Test accuracy: {accuracy}')

loaded_model = keras.models.load_model('image_classifier_model.h5')

def classify_image(image_path):
    img = image.load_img(image_path, target_size=(224, 224))
    img_array = image.img_to_array(img)
    img_array /= 255.0
    img_array = np.expand_dims(img_array, axis=0)

    predictions = loaded_model.predict(img_array)[0]
    print(np.max(predictions))
    predicted_label = categories[np.argmax(predictions)]
    if(np.max(predictions)>0.9):
      return predicted_label
    else:
      return "Not Found"
    return


vcap = cv2.VideoCapture('http://192.168.29.190:4747/video')
#if not vcap.isOpened():
#    print "File Cannot be Opened"
img_counter = 0
while(True):
    # Capture frame-by-frame
    ret, frame = vcap.read()
    #print cap.isOpened(), ret
    if frame is not None:
        # Display the resulting frame
        cv2.imshow('frame',frame)
        # Press q to close the video windows before it ends if you want
        k = cv2.waitKey(1)
        if k%256 == 27:
            # ESC pressed
            print("Escape hit, closing...")
            break
        elif k%256 == 32:
            # SPACE pressed
            img_name = "opencv_frame_{}.png".format(img_counter)
            cv2.imwrite(img_name, frame)
            print("{} written!".format(img_name))
            test_image_path = r'C:\Users\Bagavathi\Downloads\python/'
            test_image_path = test_image_path.replace("\\","/")
            test_image_path = test_image_path + img_name
            prediction = classify_image(test_image_path)
            print(f'The image is classified as: {prediction}')
           
            # img_counter += 1
    else:
        print("Frame is None")
        break

# When everything done, release the capture
vcap.release()
cv2.destroyAllWindows()
print("Video stop")
