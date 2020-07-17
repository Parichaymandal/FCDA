############################################################################################
## This code is created as a part of the study project "Floating Car Data Analysis". 
## This modules handles traffic related statistics
############################################################################################
import pandas as pd
import numpy as np
import pydeck as pdk
import folium
from aggregation import *

def waiting_cars(edgeset, threshold):
    '''
        This functions aggregate the number of distinct car wait in a road segment
        Parameters:
        ____________
        edgeset: Pandas Dataframe
            Dataframe containing the dataset
        threshold: float
            A thresold below which a car is considered to be in waiting

        Returns:
            returns dataframe after aggregation    

    '''
    # aggregate the number of distict cars tracks with speed less than the threshold for every segment
    edge_agg = edgeset[edgeset['speed'] < threshold].groupby(['st_node', 'end_node']).agg(waiting=pd.NamedAgg(column='track_id', aggfunc='nunique'))
    # merge the aggregated dataframe to the main dataframe 
    edgeset = pd.merge(edgeset, edge_agg,  how='left', left_on=['st_node','end_node'], right_on = ['st_node','end_node'])
    #return resulting dataframe
    return edgeset

def passing_cars(edgeset):
    '''
        This functions aggregate the number of distinct car passing by a road segment
        Parameters:
        ____________
        edgeset: Pandas Dataframe
            Dataframe containing the dataset

        Returns:
            returns dataframe after aggregation       
    '''
    # aggregate the number of distict cars tracks for every segment
    edge_agg = edgeset.groupby(['st_node', 'end_node']).agg(passing=pd.NamedAgg(column='track_id', aggfunc= 'nunique'))
    # merge the aggregated dataframe to the main dataframe 
    edgeset = pd.merge(edgeset, edge_agg,  how='left', left_on=['st_node','end_node'], right_on = ['st_node','end_node'])
    #return resulting dataframe
    return edgeset 

def stopped_cars(edgeset):
    '''
        This functions aggregate the number of distinct car stopped on a road segment
        Parameters:
        ____________
        edgeset: Pandas Dataframe
            Dataframe containing the dataset

        Returns:
            returns dataframe after aggregation       
    '''

    # aggregate the number of distict cars tracks with speed equals to zero for every segment
    edge_agg = edgeset[edgeset['speed'] == 0].groupby(['st_node', 'end_node']).agg(stopped=pd.NamedAgg(column='track_id', aggfunc='nunique'))
    # merge the aggregated dataframe to the main dataframe    
    edgeset = pd.merge(edgeset, edge_agg,  how='left', left_on=['st_node','end_node'], right_on = ['st_node','end_node'])
    #return resulting dataframe
    return edgeset

def plot_stat(edgeset, col = 'waiting'):

    '''
        This functions plots the aggregated data
        Parameters:
        ____________
        edgeset: Pandas Dataframe
            Dataframe containing the dataset
        col: string
            Which column want to plot    

        Returns:
            returns dataframe after aggregation       
    '''

    # add corresponding cordinates
    appendNodeCoords(edgeset)
    # find the center condinates
    coord_list= FindCenterCoords(edgeset)
    init_lat = coord_list[0]
    init_lon = coord_list[1]
    
    # set initial view of the map
    INITIAL_VIEW_STATE = pdk.ViewState(latitude=init_lat, longitude=init_lon, zoom=15, max_zoom=20, pitch=50, bearing=0)

    
    # set map color according to the values
    if col in ['waiting', 'passing', 'stopped']:
        min_val = edgeset[col].min()
        max_val = edgeset[col].max()
        edgeset[col] =  (edgeset['waiting']/(max_val - min_val))*255
        linecolor = '[color ,(color * 10) % 255,0,255]'
        pointcolor='[color , (color *10) % 255,0,255]'

    # scatter plot layer   
    scatterplot = pdk.Layer(
        "ScatterplotLayer",
        edgeset,
        radius_scale=5,
        get_position=['StNode_lon', 'StNode_lat'],
        get_fill_color=pointcolor,
        get_radius=1,
        pickable=True,
    )

  
    # line layer
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
