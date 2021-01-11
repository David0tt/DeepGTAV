from deepgtav.messages import Start, Stop, Scenario, Dataset, Commands, frame2numpy, GoToLocation, TeleportToLocation, SetCameraPositionAndRotation
from deepgtav.messages import StartRecording, StopRecording, SetClockTime, SetWeather
from deepgtav.client import Client

if __name__ == '__main__':

    client = Client()

    scenario = Scenario(drivingMode=786603, vehicle="buzzard", location=[245.23306274414062, -998.244140625, 29.205352783203125]) #automatic driving
    dataset=Dataset(location=True, time=True, exportBBox2D=True, instanceSegmentationImageColor=True) #exportIndividualStencilImages=True)

    client.sendMessage(Start(scenario=scenario, dataset=dataset))

    client.sendMessage(SetCameraPositionAndRotation(z = -20, rot_x = -30))

    count = 0
    
    while True:
        count += 1
        print("count: ", count)

        # Only record every 10th frame
        if count > 50 and count % 10 == 0:
            client.sendMessage(StartRecording())
        if count > 50 and count % 10 == 1:
            client.sendMessage(StopRecording())

        message = client.recvMessage()  

        print(message["bbox2d"])
