## 14259 CenterStage - VISION TESTING

hey guys welcome back to another tutorial

say we have AprilTagProcessor apriltag

## apriltag.detectSpecificTag(int)

this function detects only a specified exists. for detecting that specific tag's data use getSpecificTagData(int)

## Detection class
hi veer
once you detect an apriltag (or multiple) it will be in type AprilTagDetection
example of AprilTagDetection detection

detection.x, detection.y , detection.z are the coords to the apriltag
l l l l l l l l
l             l          ^ z
l             l          i
l             l     x <- o y
l             l     detection.y will face towards the camera
l l l l l l l l

all of this is stored within detection.metadata
so check that it exists using if(detection.metadata == null)



i have the measurements set to inches but that could change

## Pattern Detection

no not yet ill handle that later