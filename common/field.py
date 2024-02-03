"""
Thank you to the FROG team

https://github.com/FROG3160/2023-charged-up/blob/445c4d6225902a0fd2359cf92854632d6f3e2aaa/roborio-src/components/vision.py

"""
import logging
import os

import wpilib
from robotpy_apriltag import AprilTagFieldLayout
from wpimath.geometry import Pose3d, Rotation3d, Transform3d, Translation3d
from wpimath.units import inchesToMeters

apriltagsFilename = r"apriltags_layout.json"
# get the dir of THIS file (vision.py), go up one level (..), and use the specified filename
apriltagsLayoutPath = os.path.join(os.path.dirname(__file__), r"..", apriltagsFilename)

X_CHANGE = 1.27257
Y_CHANGE = inchesToMeters(22)
Z_CHANGE = 0.462788

RED_ALLIANCE = wpilib.DriverStation.Alliance.kRed
BLUE_ALLIANCE = wpilib.DriverStation.Alliance.kBlue

botPositionsFromTag = (
    Transform3d(Translation3d(X_CHANGE, -Y_CHANGE, -Z_CHANGE), Rotation3d(0, 0, 0)),
    Transform3d(Translation3d(X_CHANGE, 0, -Z_CHANGE), Rotation3d(0, 0, 0)),
    Transform3d(Translation3d(X_CHANGE, Y_CHANGE, -Z_CHANGE), Rotation3d(0, 0, 0)),
)

# tag list ordered by view from driverstation of the alliance color
# ( rightTagID, MiddleTagID, LeftTagID)
blueTagList = (8, 7, 6)
redTagList = (3, 2, 1)

blueGridPositions = {
    1: (8, 1),
    2: (8, 2),
    3: (8, 3),
    4: (7, 1),
    5: (7, 2),
    6: (7, 3),
    7: (6, 1),
    8: (6, 2),
    9: (6, 3),
}

redGridPositions = {
    1: (3, 1),
    2: (3, 2),
    3: (3, 3),
    4: (2, 1),
    5: (2, 2),
    6: (2, 3),
    7: (1, 1),
    8: (1, 2),
    9: (1, 3),
}


def getAlliance():
    return wpilib.DriverStation.getAlliance()


class FieldLayout:
    def __init__(self):
        self.logger = logging.getLogger("FieldLayout")
        # super().__init__(apriltagsLayoutPath)
        self.alliance = wpilib.DriverStation.Alliance.kBlue
        # set layout to be specific to the alliance end

    def getTagtoRobotTransform(self, fieldPose: Pose3d, tagID: int) -> Transform3d:
        return fieldPose - self.getTagPose(tagID)

    # get position
    def getTagRelativePosition(self, tagID: int, position: int) -> Pose3d:
        return self.getTagPose(tagID) + botPositionsFromTag[position - 1]

    def getGridRelativePosition(self, gridNum: int, position: int) -> Pose3d:
        return self.getTagRelativePosition(self.tagList[gridNum - 1], position)

    def getPosition(self, position: int) -> Pose3d:
        """Returns the field pose for the robot to use to be in front
        of the given grid position

        Args:
            position (int): Grid position, from 1 to 9 with 1 being the furthest right

        Returns:
            Pose3d: Field pose
        """
        return self.getTagRelativePosition(*self.gridPositions[position])

    # set alliance/change origin
    def setAlliance(self, alliance=getAlliance()):
        self.logger.info(f"FieldLayout.setAlliance() called with {alliance}")
        if alliance == RED_ALLIANCE:
            self.setOrigin(self.OriginPosition.kRedAllianceWallRightSide)
            self.gridPositions = redGridPositions
            self.tagList = redTagList
            self.alliance = RED_ALLIANCE
        elif alliance == BLUE_ALLIANCE:
            self.setOrigin(self.OriginPosition.kBlueAllianceWallRightSide)
            self.gridPositions = blueGridPositions
            self.tagList = blueTagList
            self.alliance = BLUE_ALLIANCE


if __name__ == "__main__":
    pass
