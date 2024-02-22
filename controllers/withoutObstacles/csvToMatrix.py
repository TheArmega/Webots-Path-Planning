import csv
import numpy

class csvToMatrix:

    def __init__(self, mapRoute):
        self.mapRoute = mapRoute

    def transform(self):
        reader = csv.reader(open(self.mapRoute, "rt"), delimiter=",")
        x = list(reader)
        result = numpy.array(x).astype("int")

        return result