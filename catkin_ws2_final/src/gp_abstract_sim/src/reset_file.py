#!/usr/bin/env python
import os
save_path = os.path.dirname(__file__)
# save_path = '/home/kholio/auto_ws/src/mybot/mybot_navigation/scripts/'
name_of_file = "last_yaw_goal" 

completeName = os.path.join(save_path, name_of_file+".txt")         
file1 = open(completeName, "w")

toFile1 = str(0.0)+ "\n"
toFile2 = str(0.0)+ "\n"
toFile3 = str(0.0)+ "\n"
toFile4 = str(0.0)+ "\n"
toFile5 = str(0.0)+ "\n"
toFile6 = str(0.0)+ "\n"
toFile7 = str(0.0)+ "\n"
toFile8 = str(0.0)+ "\n"
toFile9 = str(0.0)+ "\n"
toFile10 = str(0.0)+ "\n"
file1.write(toFile1)
file1.write(toFile2)
file1.write(toFile3)
file1.write(toFile4)
file1.write(toFile5)
file1.write(toFile6)
file1.write(toFile7)
file1.write(toFile8)
file1.write(toFile9)
file1.write(toFile10)
file1.close()