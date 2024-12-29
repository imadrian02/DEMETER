from ultralytics import YOLO

# Load the pre-trained model
model = YOLO(r'C:\Users\adria\DEMETER\model\crop.pt')

# Display model summary
model.info()




#while True:
    #model(source=r'C:\Users\adria\DEMETER\photo_1_2024-04-28_12-32-06.jpg', conf=0.01, show=True)
