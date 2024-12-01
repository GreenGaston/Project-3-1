import csv
from paraview.simple import *



timesteps = reader.GetProperty("TimestepValues")
output_csv= "C:/Users/nikit/Downloads/output.csv"
reader = vv.getReader()

with open(output_csv , mode="w", newline="") as file:
        writer= csv.writer(file)
        writer.writerow(["Timestep","X","Y","Z","Timestamp"])
        for idx in range(len(timesteps)):
            reader.UpdatePipeline(timesteps[idx])
            cloudInfo = reader.GetClientSideObject().GetOutput()
            points = cloudInfo.GetPoints()
            times=cloudInfo.GetPointData().GetArray("timestamp")
            for i in range(points.GetNumberOfPoints()):
                x,y,z=points.GetPoint(i)
                timestamp = times.GetValue(i) if times else None
                writer.writerow([timesteps[idx],x,y,z,timestamp])
print(f"Data written to {output_csv}")