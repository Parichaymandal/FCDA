import pandas as pd
import geopandas as gpd
import numpy as np
import osmnx as ox
import folium
from pyproj import Geod
from leuvenmapmatching.matcher.distance import DistanceMatcher
from leuvenmapmatching.matcher.newsonkrumm import NewsonKrummMatcher
from leuvenmapmatching.matcher.simple import SimpleMatcher
from leuvenmapmatching.map.inmem import InMemMap

def get_InMemMap(graph):
    # Leuven Map Matching is using a different internal graph structure for the street data. 
    # Therefore, the OSMnx graph needs to be transformed to the InMemMap
    streetmap = InMemMap("enviroCar", use_latlon=True, use_rtree=True, index_edges=True)
    # add nodes
    nodes = list(graph.nodes)
    for node in nodes:
        lng = graph.nodes[node]['x']
        lat = graph.nodes[node]['y']
        streetmap.add_node(node, (lat, lng))

    # add edges
    edges = list(graph.edges)
    for edge in edges:
        node_a, node_b = edge[0], edge[1]
        streetmap.add_edge(node_a, node_b)

        # exclude bi-directional edges when street is oneway
        if not graph.edges[edge]['oneway']:
            streetmap.add_edge(node_b, node_a)
        
    streetmap.purge()

    return streetmap

def match(track, graph, matcher_alg = 'DistanceMatcher',
                            max_dist=200, 
                            max_dist_init=100,
                            min_prob_norm=0.001,
                            non_emitting_length_factor=0.75,
                            obs_noise=50,
                            obs_noise_ne=75,
                            dist_noise=50,
                            non_emitting_edgeid=False):

    streetmap = get_InMemMap(graph)

    if(matcher_alg == 'DistanceMatcher'):
        matcher = DistanceMatcher(streetmap,
                            max_dist = max_dist, 
                            max_dist_init=max_dist_init,
                            min_prob_norm=min_prob_norm,
                            non_emitting_length_factor=non_emitting_length_factor,
                            obs_noise=obs_noise,
                            obs_noise_ne=obs_noise_ne,
                            dist_noise=dist_noise,
                            non_emitting_edgeid=non_emitting_edgeid)

    elif(matcher_alg == 'NewsonKrummMatcher'):
        matcher = NewsonKrummMatcher(streetmap,
                            max_dist = max_dist, 
                            max_dist_init=max_dist_init,
                            min_prob_norm=min_prob_norm,
                            non_emitting_length_factor=non_emitting_length_factor,
                            obs_noise=obs_noise,
                            obs_noise_ne=obs_noise_ne,
                            dist_noise=dist_noise,
                            non_emitting_edgeid=non_emitting_edgeid)

    elif(matcher_alg == 'SimpleMatcher'):
        matcher = SimpleMatcher(streetmap,
                            max_dist = max_dist, 
                            max_dist_init=max_dist_init,
                            min_prob_norm=min_prob_norm,
                            non_emitting_length_factor=non_emitting_length_factor,
                            obs_noise=obs_noise,
                            obs_noise_ne=obs_noise_ne,
                            dist_noise=dist_noise,
                            non_emitting_edgeid=non_emitting_edgeid)

    else:
        print('No matcher selected')
        return


    # Perform the mapmatching 
    edge_ids, last_idx = matcher.match(track)

    # Reference ellipsoid for distance
    geod = Geod(ellps='WGS84')
    proj_dist = np.zeros(len(track))

    # edgeid refers to edges id (node1_id, node2_id) where the GPS point is projected
    lat_corr, lon_corr = [], []
    lat_nodes = matcher.lattice_best
    for idx, m in enumerate(lat_nodes):
        if(idx == len(track)):
            break
        lat, lon = m.edge_m.pi[:2]
        lat_corr.append(lat)
        lon_corr.append(lon)
        # print(idx)
        _, _, distance = geod.inv(track[idx][1], track[idx][0], lon, lat)
        proj_dist[idx] += distance

        
    track_corr = np.column_stack((lat_corr, lon_corr))   
    route = compute_route(streetmap, track_corr, edge_ids)

    return edge_ids, last_idx, track_corr, route


def compute_route(graph, track_corr, edge_ids):
    geod = Geod(ellps='WGS84')
    # Compute the route coordinates
    route = []
    path_length = []
    unlinked = []

    for i in range(len(track_corr) - 1):
        if edge_ids[i] != edge_ids[i+1]:
            route.append(track_corr[i])
            route.append([graph.graph[edge_ids[i][1]][0][0], graph.graph[edge_ids[i][1]][0][1]])
            _, _, distance = geod.inv(track_corr[i][1], track_corr[i][0],
                                        graph.graph[edge_ids[i][1]][0][1], graph.graph[edge_ids[i][1]][0][0])
            path_length.append(distance)
            unlinked.append(0)

        else:
            route.append(track_corr[i])
            _, _, distance = geod.inv(track_corr[i][1], track_corr[i][0],
                                        track_corr[i+1][1], track_corr[i+1][0])
            path_length.append(distance)
            unlinked.append(0)
    # Let's not forget the last point
    # route.append(track_corr[-1])
    route = np.array(route)

    return route


def plot_html(track, track_corr=[],
             track_color="black", track_corr_color="#CD473E",
             track_size=2, track_corr_size=2,
             route_corr=[],
             route_size=2, route_corr_size=2,
             route_color="black", route_corr_color="#CD473E",
             route_opacity=.7, route_corr_opacity=.7,
             proj=False, proj_color="#CD473E", proj_size=1, proj_alpha=.5,
             show_graph=False, graph=None,
             file_name="my_map.html", save=True
             ):
    """
    

    Parameters
    ----------
    track : TYPE
        DESCRIPTION.
    track_corr : TYPE, optional
        DESCRIPTION. The default is [].
    track_color : TYPE, optional
        DESCRIPTION. The default is "black".
    track_corr_color : TYPE, optional
        DESCRIPTION. The default is "darkcyan".
    track_size : TYPE, optional
        DESCRIPTION. The default is 2.
    track_corr_size : TYPE, optional
        DESCRIPTION. The default is 2.
    route_corr : TYPE, optional
        DESCRIPTION. The default is [].
    route_size : TYPE, optional
        DESCRIPTION. The default is 2.
    route_corr_size : TYPE, optional
        DESCRIPTION. The default is 2.
    route_color : TYPE, optional
        DESCRIPTION. The default is "black".
    route_corr_color : TYPE, optional
        DESCRIPTION. The default is "darkcyan".
    route_opacity : TYPE, optional
        DESCRIPTION. The default is .6.
    route_corr_opacity : TYPE, optional
        DESCRIPTION. The default is .6.
    proj : TYPE, optional
        DESCRIPTION. The default is False.
    proj_color : TYPE, optional
        DESCRIPTION. The default is "skyblue".
    proj_size : TYPE, optional
        DESCRIPTION. The default is 1.
    proj_alpha : TYPE, optional
        DESCRIPTION. The default is 1.
    show_graph : TYPE, optional
        DESCRIPTION. The default is False.
    graph : TYPE, optional
        DESCRIPTION. The default is None.
    file_name : TYPE, optional
        DESCRIPTION. The default is "my_map.html".
    save : TYPE, optional
        DESCRIPTION. The default is True.

    Returns
    -------
    my_map : TYPE
        DESCRIPTION.

    """

    med_lat = track[len(track)//2][0]
    med_lon = track[len(track)//2][1]

    # Load map centred on central coordinates
    my_map = folium.Map(location=[med_lat, med_lon], zoom_start=13)


    # If the route is given in input, plot both (original and corrected)
    if len(route_corr) > 0:
        # add lines
        folium.PolyLine(track, color=route_color, weight=route_size, opacity=route_opacity).add_to(my_map)
        folium.PolyLine(route_corr, color=route_corr_color, weight=route_corr_size, opacity=route_corr_opacity).add_to(my_map)


    # add dots
    for i in range(len(track)):
        folium.CircleMarker(location=[track[i][0], track[i][1]],
                            radius=track_size,
                            weight=1,
                            color=track_color,
                            fill=True,
                            fill_opacity=1).add_to(my_map)
    for i in range(len(track_corr)):
        folium.CircleMarker(location=[track_corr[i][0], track_corr[i][1]],
                            radius=track_corr_size,
                            weight=1,
                            color=track_corr_color,
                            fill=True,
                            fill_opacity=1).add_to(my_map)


    # add the OSM light grey background
    folium.TileLayer('cartodbpositron').add_to(my_map)

    # plot the legend in the HTML page
    legend_html = """
    <div style="position: fixed;
                width: 210px;
                top: 10px; right: 10px;
                border: 2px solid lightgrey;
                border-radius: 4px;
                background-color: rgba(255, 255, 255, 0.85);
                z-index:9999;
                font-size: 15px; color: slategrey;
     ">
         &nbsp; <span style="font-weight: bold">Legend</span>
         <br>
             &nbsp; Original Point &nbsp;
             <i class="fa fa-circle"
                 style="float: right;
                         margin-right: 19px; margin-top: 4px;
                         color: black">
             </i>
         <br>
             &nbsp; Projected Point &nbsp;
             <i class="fa fa-circle"
                 style="float: right;
                         margin-right: 19px; margin-top: 4px;
                         color: #CD473E">
             </i>
         <br>
             &nbsp; Projection &nbsp;
             <div class="line"
                 style="float: right;
                         margin-right: 10px; margin-top: 10px;
                         width: 30px; height: 2px;
                         background-color: #CD473E">
             </div>
    </div>
    """
    my_map.get_root().html.add_child(folium.Element(legend_html))

    if save:
        my_map.save(file_name)
        # Plot in new tab
        #webbrowser.open(file_name, new=2)  # open in new tab

    return my_map