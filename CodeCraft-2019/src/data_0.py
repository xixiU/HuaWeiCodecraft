# -*- coding: utf-8 -*-
"""
Created on Sat Mar 16 09:35:21 2019

@author: ym
"""

import pandas as pd
#from dijkstra import DijkstraExtendPath
import numpy as np

class DijkstraExtendPath():
    def __init__(self, node_map):
        self.node_map = node_map
        self.node_length = len(node_map)
        self.used_node_list = []
        self.collected_node_dict = {}
    def __call__(self, from_node, to_node):
        self.from_node = from_node
        self.to_node = to_node
        self._init_dijkstra()
        return self._format_path()
    def _init_dijkstra(self):
        self.used_node_list.append(self.from_node)
        self.collected_node_dict[self.from_node] = [0, -1]
        for index1, node1 in enumerate(self.node_map[self.from_node]):
            if node1:
                self.collected_node_dict[index1] = [node1, self.from_node]
        self._foreach_dijkstra()
    def _foreach_dijkstra(self):
        if len(self.used_node_list) == self.node_length - 1:
            return
        temp_dic=self.collected_node_dict.copy()
        for key, val in temp_dic.items():  # 遍历已有权值节点
            if key not in self.used_node_list and key != self.to_node:
                self.used_node_list.append(key)
            else:
                continue
            for index1, node1 in enumerate(self.node_map[key]):  # 对节点进行遍历
                # 如果节点在权值节点中并且权值大于新权值
                if node1 and index1 in self.collected_node_dict and self.collected_node_dict[index1][0] > node1 + val[0]:
                    self.collected_node_dict[index1][0] = node1 + val[0] # 更新权值
                    self.collected_node_dict[index1][1] = key
                elif node1 and index1 not in self.collected_node_dict:
                    self.collected_node_dict[index1] = [node1 + val[0], key]
        self._foreach_dijkstra()
    def _format_path(self):
        node_list = []
        temp_node = self.to_node
        node_list.append((temp_node, self.collected_node_dict[temp_node][0]))
        while self.collected_node_dict[temp_node][1] != -1:
            temp_node = self.collected_node_dict[temp_node][1]
            node_list.append((temp_node, self.collected_node_dict[temp_node][0]))
        node_list.reverse()
        return node_list
    
class solve:
    def __init__(self,car_txt,cross_txt,road_txt):
        self.car=pd.read_csv(car_txt,sep=',|\#\(|\(|\)')
        self.cross=pd.read_csv(cross_txt,sep=',|\#\(|\(|\)')
        self.road=pd.read_csv(road_txt,sep=',|\#\(|\(|\)')
        self.node=self.cross['id'].tolist()
    def grt_map(self,one_car):
        #speed=onecar['speed']
        def speed(speed,lim_spd,length):
            return length/min(speed,lim_spd)
        node_list=[]
        #print(one_car)
        for i in range(len(self.road)):
            node_list.append(((self.road.ix[i]['from']),self.road.ix[i]['to'],speed(one_car['speed'],self.road.ix[i]['speed'],self.road.ix[i]['length'])))
            if self.road.ix[i]['isDuplex']==1:
                node_list.append((self.road.ix[i]['to'],self.road.ix[i]['from'],speed(one_car['speed'],self.road.ix[i]['speed'],self.road.ix[i]['length'])))
        return node_list
    def get_path(self,one_car):
        node=self.node
        node_map = [[0 for val in range(len(node))] for val in range(len(node))]
        #print(one_car)
        node_list=self.grt_map(one_car)
        #print(node,node_list)
        def set_node_map(node_map, node, node_list):
            for x, y, val in node_list:
                node_map[node.index(x)][node.index(y)]=  val
        set_node_map(node_map, node, node_list)
        # A -->; D
        from_node = node.index(one_car['from'])
        to_node = node.index(one_car['to'])
        #print(from_node,to_node,node_map)
        dijkstrapath = DijkstraExtendPath(node_map)
        path = dijkstrapath(from_node, to_node)
        return path
    def get_pathes(self):
        return [self.get_path(self.car.ix[i]) for i in range(len(self.car))]
    
def main_process(car_path,road_path,cross_path,answer_path):
    """
    car_path,road_path,cross_path,answer_path
    """
    Solve=solve(car_path,road_path,cross_path)
    pathes=Solve.get_pathes()
    #(carId,StartTime,RoadId...)
    result =[]
    car_list = Solve.car['id']
    planTime_list = Solve.car['planTime']
    getRoadList = lambda id :Solve.cross.loc[Solve.cross['id']==id].iloc[:,2:6].values.astype(np.int64)[0].tolist()
    for key,carvalue in enumerate(pathes):
        one_path=[]
        one_path.extend([car_list[key],planTime_list[key]])
        prevalue = carvalue[0][0]
        for current_value,_time in carvalue[1:]:
            roadId = list(set(getRoadList(prevalue+1))&set(getRoadList(current_value+1)))
            if -1 in roadId:
                roadId.remove(-1)
            one_path.extend(roadId)
            prevalue = current_value
        result.append(tuple(one_path))
    with open(answer_path,'w') as f_w:
        f_w.write('#(carId,StartTime,RoadId...)\n')
        for t in result:
            f_w.write('('+ ','.join(str(s) for s in t) +')'+ '\n')
if __name__=='__main__':
    main_process('~/Documents/code/competition/2019huawei/2019软挑-初赛-SDK/SDK/SDK_python/CodeCraft-2019/config/car.txt','~/Documents/code/competition/2019huawei/2019软挑-初赛-SDK/SDK/SDK_python/CodeCraft-2019/config/cross.txt','~/Documents/code/competition/2019huawei/2019软挑-初赛-SDK/SDK/SDK_python/CodeCraft-2019/config/road.txt','/home/xi/Documents/code/competition/2019huawei/2019软挑-初赛-SDK/SDK/SDK_python/CodeCraft-2019/src/answer.txt')
#    Solve=solve('../config/car.txt', '../config/road.txt',' ../config/cross.txt',' ../config/answer.txt')
#    pathes=Solve.get_pathes()
#    #(carId,StartTime,RoadId...)
#    result =[]
#    car_list = Solve.car['id']
#    planTime_list = Solve.car['planTime']
#    getRoadList = lambda id :Solve.cross.loc[Solve.cross['id']==id].iloc[:,2:6].values.astype(np.int64)[0].tolist()
#    for key,carvalue in enumerate(pathes):
#        one_path=[]
#        one_path.extend([car_list[key],planTime_list[key]])
#        prevalue = carvalue[0][0]
#        for current_value,_time in carvalue[1:]:
#            roadId = list(set(getRoadList(prevalue+1))&set(getRoadList(current_value+1)))
#            if -1 in roadId:
#                roadId.remove(-1)
#            one_path.extend(roadId)
#            prevalue = current_value
#        result.append(tuple(one_path))
#    with open('answer.txt','w') as f_w:
#        f_w.write('#(carId,StartTime,RoadId...)\n')
#        for t in result:
#            f_w.write('('+ ','.join(str(s) for s in t) +')'+ '\n')
    


