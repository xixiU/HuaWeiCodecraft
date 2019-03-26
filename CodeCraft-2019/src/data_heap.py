# -*- coding: utf-8 -*-
"""
Created on Sat Mar 16 09:35:21 2019

@author: ym
"""

import pandas as pd
#from dijkstra import DijkstraExtendPath
import numpy as np
from io import StringIO
import operator
import time
from dijkstra_heap import dijkstra,retype

    
class road_t_distribution:
    def __init__(self,road):
        #self.from_to={(from,to):[np.array]}
        self.from_to=dict()
        for item in road:
            self.from_to[item]=[(1,100000)]
            
    def __call__(self,road_t_list):
        #assert road_t_list[road_t_list.keys()[0]][0]==0
        #road_list={(from,to):[start,end]}
        start_time=10
        #temp_time_table= road_t_list.keys()
        point=True
        while point:
            cond,query=self.judge(start_time,road_t_list)#查询满足条件的字段
            if cond:
                self.split_renew(query)#更新字段
                return start_time
            start_time+=3
            
    def judge(self,start_time,road_t_list):
        query_dic={}
        for item in road_t_list.keys():
            temp=self.from_to[item]
            obj_=road_t_list[item]
            start_=obj_[0]+start_time
            end_=obj_[1]+start_time
            temp_cond=False
            #查询from_to
            for time_item in temp :
                if start_ >= time_item[0] and end_ <= time_item[1]:
                    temp_cond = True
                    break
            if temp_cond:
                query_dic[item]=[time_item,(start_,end_)]
            else:
                return False,{}
        return True,query_dic
    
    def split_renew(self,query_dic):
        for item in query_dic.keys():
            x1,y1=query_dic[item][0]
            x2,y2=query_dic[item][1]
            self.from_to[item].append((x1,x2))
            self.from_to[item].append((y2,y1))
            del self.from_to[item][self.from_to[item].index(query_dic[item][0])]
            

class solve:   
    def __init__(self,path='../config/'):
        start=time.time()
        car_txt=path+'car.txt'
        cross_txt=path+'cross.txt'
        road_txt=path+'road.txt'
        self.car=pd.read_csv(car_txt,sep=',|\#\(|\(|\)',engine='python')
        self.cross=pd.read_csv(cross_txt,sep=',|\#\(|\(|\)',engine='python')
        self.road=pd.read_csv(road_txt,sep=',|\#\(|\(|\)',engine='python')
        self.car.drop(['Unnamed: 0','Unnamed: 6'],axis=1,inplace=True)
        self.cross.drop(['Unnamed: 0','Unnamed: 6'],axis=1,inplace=True)
        self.road.drop(['Unnamed: 0','Unnamed: 8'],axis=1,inplace=True)
        self.getRoadList = lambda id :self.cross.loc[self.cross['id']==id].iloc[:,1:5].values.astype(np.int64)[0].tolist()
        self.get_road_from2cross = lambda cross1,cross2: list(set(self.getRoadList(int(cross1)))&set(self.getRoadList(int(cross2))))#转int 再求list
        self.cross2Road={}#tuple(cross1,cross2):roadid dict
        #得到出发点终点
        self.car['from_to']=self.car.apply(lambda item:(item['from'],item['to']),axis=1)
        #最终需要的两个属性
        self.car['pathes']=None
        self.car['time']=None
        #根据from_to聚合的car_dataframe
        self.from_to_car_dic={}
        self.from_to_pathes_dic={}                                                                                        
        self.from_to_car_num={}
        for item in self.car.from_to.unique():
            self.from_to_car_dic[item]=self.car[self.car['from_to']==item]
            self.from_to_car_num[item]=len(self.car[self.car.from_to==item])
        #根据车辆数目来确定先后顺序
        self.from_to_sorted=sorted(self.from_to_car_num.items(),key=operator.itemgetter(1),reverse=True)
        #需要确定的路径
        self.node=self.cross['id'].tolist()
        #self.node_map_dic={}
        self.node_map = [[0 for val in range(len(self.node))] for val in range(len(self.node))]
        #self.node_list=self.grt_map()
        def get_condition(dtframe):
            road=dtframe
            road_c=dict()
            road_time=dict()
            road_weight=dict()
            road_from_path=dict()
            for row_index,item in road.iterrows():
                road_c[(item['from'],item['to'])]=item['channel']
                road_time[(item['from'],item['to'])]={}
                road_weight[(item['from'],item['to'])]=item['length']
                road_from_path[(item['from'],item['to'])]=item['id']
                if item['isDuplex']:
                   road_c[(item['to'],item['from'])]=item['channel']
                   road_time[(item['to'],item['from'])]={}
                   road_weight[(item['to'],item['from'])]=item['length']/item['channel']
                   road_from_path[(item['to'],item['from'])]=item['id']
            return road_weight,road_c,road_time,road_from_path
        #self.from_to=self.car.apply(lambda item : (item['from'],item['to']),axis=1)
        self.road_dynamic_weight,self.road_condition,self.road_time,self.right_from_to_pathes=get_condition(self.road)
        #print(time.time()-start)
        start=time.time()
        for item in self.from_to_sorted:
            path=self.get_path(item)
            self.from_to_pathes_dic[item[0]]=path
            #self.car[self.car['from_to']==item]['from_to']=path
        self.road_dis=road_t_distribution(self.road_dynamic_weight.keys())
        print(time.time()-start)
        start=time.time()
        #self.get_crossTime()
        #print(time.time()-start)
        #print(self.real_car_time)
        #print(self.from_to_pathes_dic)
    
    
    
    
    
    def grt_map(self):
        #speed=onecar['speed']
        node_list=[]
        #print(one_car)
        
        for i in range(len(self.road)):
            from_=self.road['from'][i]
            to_=self.road['to'][i]
            speed_=self.road['speed'][i]
            key=(from_,to_)
            node_list.append((from_,to_,self.road_dynamic_weight[key]/speed_))
            if self.road['isDuplex'][i]==1:
                key=(to_,from_)
                node_list.append((to_,from_,self.road_dynamic_weight[key]/speed_))
        return node_list
    
    def get_path(self,from_to):
        #from_to=((from,to),num)
        print('*****************************')
        start=time.time()
        from_=from_to[0][0]
        to_=from_to[0][1]
        num=from_to[1]
        from_node = self.node.index(from_)
        to_node = self.node.index(to_)
        node=self.node
        print(time.time()-start)
        start=time.time()
        #print(one_car)
        node_list=self.grt_map()
        print(time.time()-start)
        start=time.time()
        #print(node,node_list)
        def set_node_map(node_map, node, node_list):
            for x, y, val in node_list:
                node_map[node.index(x)][node.index(y)]=  val
        set_node_map(self.node_map, node, node_list)
        print(time.time()-start)
        start=time.time()
        #print(retype(node_list))
        #print(dijkstra(retype(node_list),from_node))
        #print(to_node==35)
        path = self.translate_path(dijkstra(retype(node_list),from_node+1)[1][to_node+1])
        print(time.time()-start)
        start=time.time()
        #path = self.translate_path(dijkstrapath(from_node, to_node))
        for item in path:
            self.road_dynamic_weight[item]+=num*0.1
        print(time.time()-start)
        start=time.time()
        return path
    
    def translate_path(self,path):
        repath=[]
        for i in range(len(path)-1):
            repath.append((path[i],path[i+1]))
        return tuple(repath)
    
    def get_pathes(self):
        return [self.get_path(self.car.loc[i]) for i in range(len(self.car))]

    def get_road_from2crossRemoveEmpty(self,cross1,cross2):
        return self.right_from_to_pathes[(cross1,cross2)]
        
    def get_crossTime(self):
        self.car_cross_time={}
        self.real_car_time={}
        for key ,cross_value in self.from_to_pathes_dic.items():
            #print(key)
            self.from_to_car_dic[key].sort_values(by=['speed'],inplace=True,ascending=False) 
            index = 0
            time_window=dict()
            i=0
            cross_time=[]
            length=int(len(self.from_to_car_dic[key])/3)
            time_window={}
            for car_row in self.from_to_car_dic[key][['id','speed']].itertuples(index=False):
#                print(index)
#                print(car_row)
                
                start_time = index
                if i == 0:
                    for crosses in cross_value:
                        #print(crosses)
                        road_pandas = self.road[self.road['id']==self.get_road_from2crossRemoveEmpty(*crosses)]
                        #print(road_pandas)
    #                    print(car_row[1])
    #                    print(type(car_row[1]))
                        road_time_costing = road_pandas['length'].values[0]*1.0/min(car_row[1],road_pandas['speed'].values[0])
                        cross_time.append(tuple([start_time,start_time+road_time_costing]))
                        time_window[crosses]=tuple([start_time,start_time+road_time_costing+length])
                        start_time = start_time+road_time_costing
                    self.car_cross_time[car_row[0]]=cross_time
                    i+=1
                else:
                    self.car_cross_time[car_row[0]]=cross_time
                index+=1
            get_add_time=self.road_dis(time_window)
            #get_add_time=0
            for car_row in self.from_to_car_dic[key][['id','speed']].itertuples(index=False):
                self.real_car_time[car_row[0]]= get_add_time+self.car_cross_time[car_row[0]][0][0]
            
                

def main_process():
    """
    car_path,road_path,cross_path,answer_path
    """
    Solve=solve('../1-map-training-4/')
    #Solve=solve('../config_9/')
    with open('answer.txt','w') as f_w:
        f_w.write('#(carId,StartTime,RoadId...)\n')
        for t,ft in Solve.car[['id','from_to']].itertuples(index=False):
            #print(t,ft)
            list_=[t,np.random.randint(10,800)]
            list_.extend([Solve.right_from_to_pathes[item] for item in Solve.from_to_pathes_dic[ft]])
            f_w.write('('+ ','.join(str(s) for s in list_) +')'+ '\n')
    return Solve
if __name__=='__main__':
   #import time
   start=time.time()
   r=main_process()
   cars = r.from_to_car_dic
   dicts = r.from_to_pathes_dic
   print(time.time()-start)
   
