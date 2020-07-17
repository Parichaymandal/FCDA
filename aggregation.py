import osmapi as osm
import numpy as np
import pandas as pd
#import plotly as pt
import osmapi as osm
#import plotly.graph_objects as go
#import plotly.express as px
import pydeck as pdk
import folium
import os
import datetime
api = osm.OsmApi()

def aggregateStatsFromNodes(edgeset, col = 'speed'):
    if col == 'speed':
        edge_agg = edgeset.groupby(['st_node', 'end_node']).agg(mean_speed=pd.NamedAgg(column='speed', aggfunc=np.mean),
                                                               min_speed=pd.NamedAgg(column='speed', aggfunc=min),
                                                               max_speed=pd.NamedAgg(column='speed', aggfunc=max))
        edgeset = pd.merge(edgeset, edge_agg,  how='left', left_on=['st_node','end_node'], right_on = ['st_node','end_node'])
        edgeset = edgeset.drop(['speed'], axis=1).drop_duplicates()
        return edgeset
    if col == 'co2':
        edge_agg = edgeset.groupby(['st_node', 'end_node']).agg(mean_co2=pd.NamedAgg(column='CO2', aggfunc=np.mean),
                                                               min_co2=pd.NamedAgg(column='CO2', aggfunc=min),
                                                               max_co2=pd.NamedAgg(column='CO2', aggfunc=max))
        edgeset = pd.merge(edgeset, edge_agg,  how='left', left_on=['st_node','end_node'], right_on = ['st_node','end_node'])
        edgeset = edgeset.drop(['CO2'], axis=1).drop_duplicates()
        return edgeset

def appendNodeCoords(edgeset):
    arrStartNodelat = np.array([])
    arrStartNodelon = np.array([])
    arrEndNodelat = np.array([])
    arrEndNodelon = np.array([])

    
    for row in edgeset.itertuples():
        
        stNode = api.NodeGet(row.st_node)
        arrStartNodelat = np.append(arrStartNodelat, stNode["lat"])
        arrStartNodelon = np.append(arrStartNodelon, stNode["lon"])
    
        endNode = api.NodeGet(row.end_node)
        arrEndNodelat = np.append(arrEndNodelat,endNode["lat"])
        arrEndNodelon = np.append(arrEndNodelon, endNode["lon"])
    
    edgeset['StNode_lat']=arrStartNodelat
    edgeset['StNode_lon']=arrStartNodelon
    edgeset['EndNode_lat']=arrEndNodelat
    edgeset['EndNode_lon']=arrEndNodelon
    
    return edgeset    


def FindCenterCoords(edgeset):
    min_lat = edgeset['StNode_lat'].min()
    max_lat = edgeset['StNode_lat'].max()
    min_lon = edgeset['StNode_lon'].min()
    max_lon = edgeset['StNode_lon'].max()
    
    
    init_lat = (min_lat+max_lat)/2
    init_long = (min_lon+max_lon)/2
    
    coord_list = []
    coord_list.append(init_lat)
    coord_list.append(init_long)
    return coord_list

def plotAggregatedStatistics(edgeset, col = 'speed'):
    FindCenterCoords(edgeset)
    coord_list= FindCenterCoords(edgeset)
    init_lat = coord_list[0]
    init_lon = coord_list[1]
    
    INITIAL_VIEW_STATE = pdk.ViewState(latitude=init_lat, longitude=init_lon, zoom=15, max_zoom=20, pitch=50, bearing=0)

    
    if col == 'speed':
        pointcolor='[mean_speed < 10 ? 50 : (mean_speed < 20 ? 100 :(mean_speed < 30 ? 150:200)),0,0,255]' 
        linecolor='[0,mean_speed < 10 ? 50 : (mean_speed < 20 ? 100 :(mean_speed < 30 ? 150:200)),0,255]'
    
    if col == 'speedLimits':
        #Red = avg speed > speed limit
        #Green = avg speed < speed limit
        #Blue = avg speed = speed limit
        #edgeset["DiffBetweenSpeedLimit"] = edgeset['mean_speed'] - edgeset['speed_limit']
        min_val = edgeset['DiffBetweenSpeedLimit'].min()
        max_val = edgeset['DiffBetweenSpeedLimit'].max()
        edgeset['color'] =  (edgeset['DiffBetweenSpeedLimit']/(max_val - min_val))*255
        
        pointcolor='[DiffBetweenSpeedLimit > 0? 255 : 0 ,DiffBetweenSpeedLimit < 0 ? 255 :0, DiffBetweenSpeedLimit == 0 ? 255 : 0,255]'
        linecolor='[DiffBetweenSpeedLimit > 0? 255 : 0 ,DiffBetweenSpeedLimit < 0 ? 255 :0, DiffBetweenSpeedLimit == 0 ? 255 : 0,255]'
        #linecolor = '[DiffBetweenSpeedLimit > 0? color : 0 ,DiffBetweenSpeedLimit < 0 ? color :0, DiffBetweenSpeedLimit == 0 ? 255 : 0,255]'
    
    if col == 'co2':
        min_val = edgeset['mean_co2'].min()
        max_val = edgeset['mean_co2'].max()
        edgeset['color'] =  (edgeset['mean_co2']/(max_val - min_val))*255
        linecolor = '[color,0,0,255]'
        pointcolor='[color,0,0,255]'

       
    scatterplot = pdk.Layer(
        "ScatterplotLayer",
        edgeset,
        radius_scale=5,
        get_position=['StNode_lon', 'StNode_lat'],
        get_fill_color=pointcolor,
        get_radius=1,
        pickable=True,
    )

  
    
    line_layer = pdk.Layer(
        "LineLayer",
        edgeset,
        get_source_position=['StNode_lon', 'StNode_lat'],
        get_target_position=['EndNode_lon', 'EndNode_lat'],
        get_color= linecolor,
        get_width=10,
        highlight_color=[255, 255, 0],
        picking_radius=10,
        auto_highlight=True,
        pickable=True,
    )

    layers = [line_layer,scatterplot]
    MAPBOX_KEY = "pk.eyJ1IjoibXByZW1hc2kiLCJhIjoiY2s5NDFueDhyMDFpODNnbjNoNzM1eWhvcCJ9.CqjZdNZJ4h8aejMWX4ZObA"
    r = pdk.Deck(layers=layers, initial_view_state=INITIAL_VIEW_STATE,mapbox_key = MAPBOX_KEY)
    r.to_html("line_layer.html", iframe_width=900)


def TimeBasedAggreagating(concatTracks, hourly, daily, monthly, pTo = False):
    concatTracks['time'] = pd.to_datetime(concatTracks['time'])
    concatTracks.dtypes
    
    filterTracks = concatTracks 
    
    if hourly:
        if pTo:
            filterTracks['hour'] = filterTracks['time'].dt.hour 
            maskHour = (filterTracks['hour'] >= hourly) & (filterTracks['hour'] <= pTo) 
            #print(filteredConcatTracksDayTime)
        
        else:
            filterTracks['hour'] = filterTracks['time'].dt.hour 
            maskHour = (filterTracks['hour'] == hourly)
            filterTracks = filterTracks.loc[maskHour]
        
    if daily:
        if pTo:
            filterTracks['weekday'] = filterTracks['time'].dt.dayofweek 
            maskDay = (filterTracks['weekday'] >= daily) & (filterTracks['weekday'] <= pTo)
            filterTracks = filterTracks.loc[maskDay]
        else:
            filterTracks['weekday'] = filterTracks['time'].dt.dayofweek 
            maskDay = (filterTracks['weekday'] == daily)
            filterTracks = filterTracks.loc[maskDay]
        
    if monthly: 
        if pTo:
            filterTracks['month'] = filterTracks['time'].dt.month 
            maskMonth = (filterTracks['month'] >= monthly) & (filterTracks['month'] <= pTo)
            filterTracks = filterTracks.loc[maskMonth]
        else:
            filterTracks['month'] = filterTracks['time'].dt.month 
            maskMonth = (filterTracks['month'] == monthly)
            filterTracks = filterTracks.loc[maskMonth]
    
    return filterTracks

