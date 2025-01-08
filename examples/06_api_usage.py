from nova.api import models


def main():
    pose = models.Pose2(position=[10, 20, 30], orientation=[1, 2, 3])
    print(pose)


if __name__ == "__main__":
    main()
