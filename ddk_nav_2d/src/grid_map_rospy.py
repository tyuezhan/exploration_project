#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid


class GridMap:
  def __init__(self):
    self.mLethalCost = 0.0

  def update(self, grid):
    self.mOccupancyGrid = grid
    self.mMapWidth = self.mOccupancyGrid.info.width
    self.mMapHeight = self.mOccupancyGrid.info.height

  def getWidth(self):
    return self.mMapWidth

  def getHeight(self):
    return self.mMapHeight

  def getSize(self):
    return self.mMapWidth * self.mMapHeight

  def getResolution(self):
    return self.mOccupancyGrid.info.resolution

  def getOriginX(self):
    return self.mOccupancyGrid.info.origin.position.x

  def getOriginY(self):
    return self.mOccupancyGrid.info.origin.position.y

  def getLethalCost(self):
    return self.mLethalCost

  def setLethalCost(self, c):
    self.mLethalCost = c

  def getMap(self):
    return self.mOccupancyGrid

  # Get the array index from the given x,y coordinates
  def getIndex(self, x, y):
    if (x >= self.mMapWidth or y >= self.mMapHeight):
      return False
    i = int(y * self.mMapWidth + x)
    return i

  # Get the x,y coordinates from the given array index
  def getCoordinates(self, i):
    if (i >= self.mMapWidth * self.mMapHeight):
      return False, False
    y = i // self.mMapWidth
    x = i % self.mMapWidth
    return x, y

  # Index based methods
  def getData(self, index):
    if (index < self.mMapWidth * self.mMapHeight):
      return self.mOccupancyGrid.data[index]
    else:
      return -1

  def setData(self, index, value):
    if (index >= self.mMapWidth * self.mMapHeight):
      return False
    self.mOccupancyGrid.data = list(self.mOccupancyGrid.data)
    self.mOccupancyGrid.data[index] = value
    self.mOccupancyGrid.data = tuple(self.mOccupancyGrid.data)
    return True

  def isFree(self, index):
    value = self.getData(index)
    if (value >= 0 and value < self.mLethalCost):
      return True
    return False

  def isObstacle(self, index):
    value = self.getData(index)
    if (value >= self.mLethalCost):
      return True
    return False

  def isFrontier(self, index):
    y = int(index // self.mMapWidth)
    x = int(index % self.mMapWidth)
    if(self.getDataByCoord(x-1, y-1) == -1):
      return True
    if(self.getDataByCoord(x-1, y) == -1):
      return True
    if(self.getDataByCoord(x-1, y+1) == -1):
      return True
    if(self.getDataByCoord(x, y-1) == -1):
      return True
    if(self.getDataByCoord(x, y+1) == -1):
      return True
    if(self.getDataByCoord(x+1, y-1) == -1):
      return True
    if(self.getDataByCoord(x+1, y) == -1):
      return True
    if(self.getDataByCoord(x+1, y+1) == -1):
      return True
    return False

  # Gets indices of all free neighboring cells with given offset
  def getFreeNeighbors(self, index, offset=1):
    neighbors = []
    if(offset < 0):
      offset = offset * -1
    y = index // self.mMapWidth
    x = index % self.mMapWidth

    for i in range(-offset, offset+1):
      for j in range(-offset, offset+1):
        neighbor = self.getIndex(x+i, y+j)
        if (neighbor != False) and self.isFree(neighbor):
          neighbors.append(neighbor)

    return neighbors

  # /** Gets indices of all neighboring cells */
  def getNeighbors(self, index, diagonal=False):
    neighbors = []
    y = index // self.mMapWidth
    x = index % self.mMapWidth
    neighbor = self.getIndex(x-1, y)
    if neighbor != False:
      neighbors.append(neighbor)
    neighbor = self.getIndex(x+1, y)
    if neighbor != False:
      neighbors.append(neighbor)
    neighbor = self.getIndex(x, y-1)
    if neighbor != False:
      neighbors.append(neighbor)
    neighbor = self.getIndex(x, y+1)
    if neighbor != False:
      neighbors.append(neighbor)

    if diagonal:
      neighbor = self.getIndex(x-1, y-1)
      if neighbor != False:
        neighbors.append(neighbor)
      neighbor = self.getIndex(x-1, y+1)
      if neighbor != False:
        neighbors.append(neighbor)
      neighbor = self.getIndex(x+1, y-1)
      if neighbor != False:
        neighbors.append(neighbor)
      neighbor = self.getIndex(x+1, y+1)
      if neighbor != False:
        neighbors.append(neighbor)

    return neighbors

  def getDataByCoord(self, x, y):
    if (x < 0 or x >= self.mMapWidth or y < 0 or y >= self.mMapHeight):
      return -1
    else:
      return self.mOccupancyGrid.data[int(y*self.mMapWidth + x)]

  def setDataByCoord(self, x, y, value):
    if (x < 0 or x >= self.mMapWidth or y < 0 or y >= self.mMapHeight):
      return False
    self.mOccupancyGrid.data = list(self.mOccupancyGrid.data)
    self.mOccupancyGrid.data[int(y*self.mMapWidth + x)] = value
    # self.mOccupancyGrid.data = tuple(self.mOccupancyGrid.data)
    return True

  def isFreeByCoord(self, x, y):
    value = self.getDataByCoord(x, y)
    if (value >= 0 and value < self.mLethalCost):
      return True
