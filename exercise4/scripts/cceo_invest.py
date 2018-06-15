from sklearn import linear_model
import pandas as pd
import numpy as np
from heapq_max import *


class DataError(Exception):
    def __init__(self, message):
        super().__init__(message)


class LinearFunction:
    def __init__(self, k, n):
        self.k = k
        self.n = n

    @classmethod
    def fromSlopePoint(cls, slope, point):
        x, y = point
        return cls(slope, y - slope * x)

    @classmethod
    def fromAngle(cls, angle, n):
        return cls(np.tan(angle), n)

    @classmethod
    def fromAnglePoint(cls, angle, point):
        x, y = point
        slope = np.tan(angle)
        return cls(slope, y - x*slope)

    def calc(self, x):
        return self.k * x + self.n


class Cceo:
    def __init__(self, color):
        self.color = color
        self.data = {'day': list(), 'value':list()}
        self.slope = None

            
    def addReading(self, day, value):
	print(self.data['day'])
        if day in self.data['day']:
            return
        self.data['day'].append(day)
        self.data['value'].append(value)

    def setSlope(self, slope):
        self.slope = slope

    def calculateValue(self):
        if self.slope is None and len(self.data["day"]) >= 2:
            df = pd.DataFrame(data=self.data)
            x = df['day']
            y = df['value']
            x = x.values.reshape(len(x), 1)
            y = y.values.reshape(len(y), 1)
            regr = linear_model.LinearRegression()
            try:
                regr.fit(x, y)
                print(float(regr.predict(7)))
                return float(regr.predict(7))
            except:
                #default value
                print("regression failed for some reason")
                return -100
                #
        elif self.slope is not None and len(self.data["day"]) >= 1:
            fp = (self.data['day'][0], self.data['value'][0])
            f = LinearFunction.fromSlopePoint(self.slope, fp)
            return f.calc(7)
        elif self.slope is not None and len(self.data["day"]) == 0:
            fp = (3, 1)
            f = LinearFunction.fromSlopePoint(self.slope, fp)
            print("Oopsie")
            return f.calc(7)
        else:
            print("no slope or 0 data points")
            return -(2 ** 16) + 1


class Investor:
    def __init__(self):
        self.q = []
        self.banks = []
        self.isQueued = False

    def indexOf(self, color):
        for i in range(len(self.banks)):
            if self.banks[i].color == color:
                return i
        return -1

    def addCceo(self, cceo):
        #if self.indexOf(cceo.color) >= 0:
            #raise DataError("{} cceo already added".format(cceo.color))
        self.banks.append(cceo)
        print(cceo.color)

    def addReading(self, color, day, value):
        index = self.indexOf(color)
        if index < 0:
            raise DataError("{} cceo doesn't exist".format(color))
        self.banks[index].addReading(day, value)

    def addSlope(self, color, slope):
        index = self.indexOf(color)
        if index < 0:
            raise DataError("{} cceo doesn't exist".format(color))
        self.banks[index].setSlope(slope)

    def createQ(self):
        self.q = []
        for bank in self.banks:
            entry = (bank.calculateValue(), bank)
            heappush_max(self.q, entry)
        self.isQueued = True

    def getNextBank(self):
        if not self.isQueued:
            self.createQ()
        if len(self.q) == 0:
            return False
        nextBank = heappop_max(self.q)
        return nextBank[1].color
