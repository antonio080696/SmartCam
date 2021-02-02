######## Image Object Detection Using Tensorflow-trained Classifier #########
# Autor: Antonio Manuel Pérez Peña

## Some of the code is copied from Google's example at
## https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb

# Import packages
import os
import cv2
import numpy as np
import tensorflow.compat.v1 as tf
import sys
from gtts import gTTS
import pygame
import pygame
import ssl
import paho.mqtt.client


# Import utilites
from utils import label_map_util
from utils import visualization_utils as vis_util

# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("..")

#Nombre de la carpeta en la que se encuentr el modelo a usar
MODEL_NAME = 'inference_graph'
#PATH donde se encuente la imagen a procesar
IMAGE_NAME = 'esp32-cam.jpg'

broker_address = "localhost"
broker_port = 1883
topic = "prueba"

#Conexion con el cliente usando MQTT
def on_connect(client, userdata, flags, rc):
 print("Connected with result code " + str(rc))
 print("UserData= " + str(userdata))
 print("flags= " + str(flags))
 print("")
 client.subscribe(topic)

#Gestiona la recepcion de mensajes mediante MQTT
def on_message(client, userdata, message):
 print("Mensaje recibido=", str(message.payload.decode("utf-8")))
 #Si recibimos un 1 significa que debe comenzar el procesamiento
 if str(message.payload.decode("utf-8")) == str(1):
     
    # Grab path to current working directory
    CWD_PATH = os.getcwd()

    # Path to frozen detection graph .pb file, which contains the model that is used
    # for object detection.
    PATH_TO_CKPT = os.path.join(CWD_PATH, MODEL_NAME, 'frozen_inference_graph.pb')

    # Path to label map file
    PATH_TO_LABELS = os.path.join(CWD_PATH, 'training', 'labelmap.pbtxt')

    # Path to image
    PATH_TO_IMAGE = os.path.join(CWD_PATH, IMAGE_NAME)

    # Number of classes the object detector can identify
    NUM_CLASSES = 11

    # Load the label map.
    # Label maps map indices to category names, so that when our convolution
    # network predicts `5`, we know that this corresponds to `king`.
    # Here we use internal utility functions, but anything that returns a
    # dictionary mapping integers to appropriate string labels would be fine
    label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
    categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES,
                                                                use_display_name=True)
    category_index = label_map_util.create_category_index(categories)

    # Load the Tensorflow model into memory.
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')

        sess = tf.Session(graph=detection_graph)

    # Define input and output tensors (i.e. data) for the object detection classifier

    # Input tensor is the image
    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

    # Output tensors are the detection boxes, scores, and classes
    # Each box represents a part of the image where a particular object was detected
    detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

    # Each score represents level of confidence for each of the objects.
    # The score is shown on the result image, together with the class label.
    detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
    detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

    # Number of objects detected
    num_detections = detection_graph.get_tensor_by_name('num_detections:0')

    # Load image using OpenCV and
    # expand image dimensions to have shape: [1, None, None, 3]
    # i.e. a single-column array, where each item in the column has the pixel RGB value
    image = cv2.imread(PATH_TO_IMAGE)
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_expanded = np.expand_dims(image_rgb, axis=0)

    # Perform the actual detection by running the model with the image as input
    (boxes, scores, classes, num) = sess.run(
        [detection_boxes, detection_scores, detection_classes, num_detections],
        feed_dict={image_tensor: image_expanded})

    # Draw the results of the detection (aka 'visulaize the results')
    # Guardamos todos los datos referentes a cada objeto
    cantidad = []
    objetos = []
    #Arrays para los objetos en cada lado de la imagen
    objetosIzquierda = []
    objetosCentro = []
    objetosDerecha = []
    #Posibles clases que vamos a encontrar
    posiblesClases = [1, 2, 3, 4, 6, 10, 13, 15, 17, 18, 64]
    #Variables para llevar la cuenta de los bucles
    r = 0
    k = 0
    z = 0

    # Seleccionamos todos aquellos objetos detectados que tengan mas de un 60% exactitud
    indices = np.where(scores[0] > 0.60000000)[0]
    #Traducción del identificador a el nombre real para formar la cadena
    Clases = {1: "persona", 2: "bicicleta", 3: "coche", 4: "moto", 6: "autobús", 10: "semáforo",
              13: "señal de stop", 15: "banco para sentarse", 17: "gato", 18: "Perro", 64: "papelera"}
    #Disccionarios usados para el recuento de los objetos encontrados en cada lado de la imagen
    ClasesIzquierda = {1: 0, 2: 0, 3: 0, 4: 0, 6: 0, 10: 0,
              13: 0, 15: 0, 17: 0, 18: 0, 64: 0}
    ClasesCentro = {1: 0, 2: 0, 3: 0, 4: 0, 6: 0, 10: 0,
              13: 0, 15: 0, 17: 0, 18: 0, 64: 0}
    ClasesDerecha = {1: 0, 2: 0, 3: 0, 4: 0, 6: 0, 10: 0,
              13: 0, 15: 0, 17: 0, 18: 0, 64: 0}

    cadena = ""
    #Recorremos todos los objetos encontrados y almacenamos sus datos de una forma adecuada
    for i in indices:
        if classes[0][i] in posiblesClases:
            box = boxes[0][i]
            score = scores[0][i]
            clase = classes[0][i]
            objeto = [clase, score, box, cantidad]
            objetos.append(objeto)
    print(objetos)
    #Recorremos todos los objetos almacenados en la funcion anterior y los colocamos segun su posicion en la imagen
    for i in objetos:
        #Si esta a la izquierda
        if i[2][1] < 0.43 and i[2][3] < 0.43:
            s = 0
            valorActual = ClasesIzquierda.get(int(i[0]))
            ClasesIzquierda[int(i[0])] = valorActual + 1
            #Comprobamos el numero de objetos que hay almacenados hasta el momento en el array y si ya hay alguno el numero de dicho objeto aumenta
            if valorActual > 0:
                for j in objetosIzquierda:
                    if j[0] == i[0]:
                        objetosIzquierda[s][3] = valorActual + 1
                    s = s+1
            else:
                i[3] = valorActual+1
                objetosIzquierda.append(i)
        #Si esta en el centro
        elif 0.43 < i[2][1] < 0.56:
            s = 0
            valorActual = ClasesCentro.get(int(i[0]))
            ClasesCentro[int(i[0])] = valorActual + 1
            ind = int(i[0])
            if valorActual > 0:
                for j in objetosCentro:
                    if j[0] == i[0]:
                        objetosCentro[s][3] = valorActual + 1
                    s = s+1
            else:
                i[3] = valorActual+1
                objetosCentro.append(i)
        #Si esta a la derecha
        elif i[2][1] > 0.56:
            s = 0
            valorActual = ClasesDerecha.get(int(i[0]))
            ClasesDerecha[int(i[0])] = valorActual + 1
            if valorActual > 0:
                for j in objetosCentro:
                    if j[0] == i[0]:
                        objetosCentro[s][3] = valorActual + 1
                    s = s+1
            else:
                i[3] = valorActual+1
                objetosDerecha.append(i)
        #Si esta en el centro
        elif i[2][1] < 0.43 and i[2][3] > 0.43:
            s = 0
            valorActual = ClasesCentro.get(int(i[0]))
            ClasesCentro[int(i[0])] = valorActual + 1
            if valorActual > 0:
                for j in objetosCentro:
                    if j[0] == i[0]:
                        objetosCentro[s][3] = valorActual + 1
                    s = s+1
            else:
                i[3] = valorActual+1
                objetosCentro.append(i)
    #Comenzamos a formar la cadena que vamos a devolver como resultado
    for i in objetosIzquierda:
        if r == 0:
            cadena = " A su izquierda: " + str(i[3]) + " " + Clases.get(int(i[0]))
        elif r > 0 and r < len(objetosIzquierda) - 1:
            cadena = cadena + " , " + str(i[3]) + " " + Clases.get(int(i[0]))
        elif r == len(objetosIzquierda) - 1:
            cadena = cadena + " y " +  str(i[3]) + " " + Clases.get(int(i[0])) + " . "
        r = r + 1

    for i in objetosCentro:
        if k == 0:
            cadena = cadena + " En el centro: " + str(i[3]) + " " + Clases.get(int(i[0]))
        elif k > 0 and k < len(objetosCentro) - 1:
            cadena = cadena + " , " + str(i[3]) + " " + Clases.get(int(i[0]))
        elif k == len(objetosCentro) - 1:
            cadena = cadena + " y " + str(i[3]) + " " + Clases.get(int(i[0])) + " . "
        k = k + 1

    for i in objetosDerecha:
        if z == 0:
            cadena = cadena + " y a la derecha: " + str(i[3]) + " " + Clases.get(int(i[0]))
        elif z > 0 and z < len(objetosDerecha) - 1:
            cadena = cadena + " , " + str(i[3]) + " " + Clases.get(int(i[0]))
        elif z == len(objetosDerecha) - 1:
            cadena = cadena + " y " + str(i[3]) + " " + Clases.get(int(i[0])) + " . "
        z = z + 1
    #Publicamos un 2 en el topic para avisar al cliente de que puede enviar otra imagen
    client.publish(topic, "2")
    #Generamos el archivo de sonido
    tts = gTTS(cadena, lang='es-es', slow= False)
    tts.save("Resultado.mp3")
    #Ejecutamos el asistente de reproduccion de archivos .mp3
    pygame.mixer.init(24000)
    pygame.mixer.music.load("Resultado.mp3")
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy() == True:
         continue

client = mqtt.Client('Cliente1', userdata="Servidor") 
client.on_connect = on_connect 
client.on_message = on_message 
client.connect(broker_address, broker_port, 60) 
client.loop_forever()

