#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Date    : 2019-03-18 22:15:19
# @Author  : yuan
# @Version : 1.0.0
# @describe: 可视化

from matplotlib import pyplot as plt
import networkx as nx
import pandas as pd
from io import StringIO
import numpy as np
import json

import flask
import networkx as nx
from networkx.readwrite import json_graph

plt.rcParams['font.sans-serif']=['SimHei'] #用来正常显示中文标签
plt.rcParams['axes.unicode_minus']=False #用来正常显示负号

def processFirstLine(filepath='/home/xi/Documents/code/competition/2019huawei/1-map-training-1/cross.txt'):
    with open(filepath,'r') as f_r:
        data = f_r.read()
    data = data.replace('(','')
    data = data.replace(')','')
    data = data.replace('#','')
    return pd.read_csv(StringIO(data))



edges=[]
edgeandnode=[]
cross_data = processFirstLine('/home/xi/Documents/code/competition/2019huawei/1-map-training-1/1-map-exam-1')
cross_id = cross_data['id'].values
for cross in cross_id:
    road_list = set(cross_data[cross_data['id']==cross].values[0])#排除多个-1的情形
    if -1 in road_list:
        road_list.remove(-1)
    for road in road_list:
        #路径存在交叉点，并且不是原来的交点
        if not cross_data[cross_data['roadId']==road].empty and cross_data[cross_data['roadId']==road]['id'].values[0]!=cross:
            cross2 = cross_data[cross_data['roadId']==road]['id'].values[0]
            if not [cross,cross2] in edges and not[cross2,cross] in edges:
                edgeandnode.append([cross,road,cross2])
                edges.append([cross,cross2])

        if not cross_data[cross_data['roadId.1']==road].empty and cross_data[cross_data['roadId.1']==road]['id'].values[0]!=cross:
            cross2 = cross_data[cross_data['roadId.1']==road]['id'].values[0]
            if not [cross,cross2] in edges and not[cross2,cross] in edges:
                edgeandnode.append([cross,road,cross2])
                edges.append([cross,cross2])

        if not cross_data[cross_data['roadId.2']==road].empty and cross_data[cross_data['roadId.2']==road]['id'].values[0]!=cross:
            cross2 = cross_data[cross_data['roadId.2']==road]['id'].values[0]
            if not [cross,cross2] in edges and not[cross2,cross] in edges:
                edgeandnode.append([cross,road,cross2])
                edges.append([cross,cross2])

        if not cross_data[cross_data['roadId.3']==road].empty and cross_data[cross_data['roadId.3']==road]['id'].values[0]!=cross:
            cross2 = cross_data[cross_data['roadId.3']==road]['id'].values[0]
            if not [cross,cross2] in edges and not[cross2,cross] in edges:
                edgeandnode.append([cross,road,cross2])
                edges.append([cross,cross2])
            
G=nx.Graph()

medge_labels ={}
for value in edgeandnode:
    medge_labels[tuple([value[0],value[2]])]=value[1]
# print(medge_labels)
plt.figure()
G.add_edges_from(edges)
pos = nx.spring_layout(G)
plt.title('路径图')
nx.draw(G,pos,edge_color='black',width=2,linewidths=2,node_size=500,node_color='pink',alpha=0.9,labels={node:node for node in G.nodes()})
nx.draw_networkx_edge_labels(G,pos,edge_labels=medge_labels,font_color='red')
plt.axis('off')
plt.show()


# from networkx.readwrite import json_graph

# for n in G:
#     G.node[n]['name'] = str(n)

# d = json_graph.node_link_data(G)
# for index,node in enumerate(d['nodes']):
#     d['nodes'][index]['id']=str(d['nodes'][index]['id'])
# for index,value in enumerate(d['links']):
#     print(d['links'][index]['source'])
#     d['links'][index]['source']=str(d['links'][index]['source'])
#     d['links'][index]['target']=str(d['links'][index]['target'])

# json.dump(d, open('/home/xi/Desktop/docu/code/competition/2019huawei/2019软挑-初赛-SDK/SDK/SDK_python/CodeCraft-2019/src/force/force.json','w'))
# print('Wrote node-link JSON data to force/force.json')

# app = flask.Flask(__name__, static_folder="force")

# @app.route('/<path:path>')
# def static_proxy(path):
#     return app.send_static_file(path)

# print('\nGo to http://localhost:8000/force.html to see the example\n')
# app.run(port=8000)
# edges=[['A','B'],['B','C'],['B','D']]
# G.add_edges_from(edges)
# pos = nx.spring_layout(G)
# plt.figure()    
# nx.draw(G,pos,edge_color='black',width=1,linewidths=1,node_size=500,node_color='pink',alpha=0.9,labels={node:node for node in G.nodes()})
# nx.draw_networkx_edge_labels(G,pos,edge_labels={('A','B'):'AB',('B','C'):'BC',('B','D'):'BD'},font_color='red')
# plt.axis('off')
# plt.show()
