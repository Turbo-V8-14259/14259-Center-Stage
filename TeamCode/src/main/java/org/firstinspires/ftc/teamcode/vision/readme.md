## 14259 CenterStage - VISION TESTING

hey guys welcome back to another tutorial

say we have AprilTagProcessor apriltag

## apriltag.detectSpecificTag(int)

this function detects only a specified tag exists. for detecting that specific tag's data use getSpecificTagData(int)

## Detection class
hi veer

once you detect an apriltag (or multiple) it will be in type AprilTagDetection

example of AprilTagDetection detection

detection.ftcpose.x, detection.ftcpose.y , detection.ftcpose.z are the coords to the apriltag

detection.x will go from right to left

detection.z will go from up to down

detection.y will face towards the camera

all of this is stored within detection.metadata

so check that it exists using if(detection.metadata == null)

i have the measurements set to inches but that could change

## Pattern Detection

no not yet ill handle that later