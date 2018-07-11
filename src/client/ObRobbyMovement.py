from timeit import default_timer as timer
from multiprocessing.pool import ThreadPool
import random
from common import header

class ObRobbyMovement(object):
    PROCEED = 0
    AVOID = 1
    def __init__(self, client, debug = False):
        self.client = client
        self.debug = debug
        self.lastActionTime =  timer()
        self.newActionList = []
        self.previousActions = [header.CaptureVideoCode.REQUEST_MOVE_RIGHT,]
        self.isProceeding = True

    def move(self, action):
        currTime = timer()
        self.newActionList.append(action)
        totalVote = len(self.newActionList)
        avoidVote = sum(self.newActionList)
        proceedVote = totalVote - avoidVote
        if currTime - self.lastActionTime > 0.2 and totalVote > 0:
            # if avoid confidence is more than 60%
            if avoidVote/totalVote > 0.6:
                self.avoid()
                self.isProceeding =  False
            else:
                self.proceed()
                self.isProceeding = True
            self.lastActionTime =  timer()
            self.newActionList.clear()

    def avoid(self):
        currentAction = self.previousActions[-1]

        if self.isProceeding:
            randomPreviousAction = random.choice(self.previousActions)
            currentAction = header.CaptureVideoCode.REQUEST_MOVE_LEFT
            if randomPreviousAction == currentAction:
                currentAction = header.CaptureVideoCode.REQUEST_MOVE_RIGHT

        if not self.debug:
            self.client.sendReq(currentAction, 1, 0.1)
        else:
            print("Avoid: move %d" % (currentAction))
        
        self.previousActions.append(currentAction)

    def proceed(self):
        if not self.debug:
            self.client.sendReq(header.CaptureVideoCode.REQUEST_MOVE_FORWARD, 1, 0.5)
        else:
            print("Proceed")

