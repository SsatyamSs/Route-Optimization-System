import streamlit as st
import osmnx as ox
import networkx as nx
import folium
from streamlit_folium import st_folium
from math import sqrt
import plotly.express as px
import pandas as pd

st.set_page_config(page_title="Emergency Route Optimization", layout="wide")
st.title("Emergency Route Optimization")

# Dehradun example start/end coordinates
start_coords = (30.3165, 78.0322)  # Dehradun center
end_coords = (30.3544, 78.0800)    # Another point in Dehradun

# Euclidean heuristic for A*
def euclidean_heuristic(u, v, G):
    x1, y1 = G.nodes[u]['x'], G.nodes[u]['y']
    x2, y2 = G.nodes[v]['x'], G.nodes[v]['y']
    return sqrt((x1 - x2)**2 + (y1 - y2)**2)

@st.cache_data(show_spinner=True)
def get_route_map(start_coords, end_coords):
    # Load graph from start point
    G = ox.graph_from_point(start_coords, dist=3000, network_type='drive')

    # Find nearest nodes to start and end points
    start_node = ox.distance.nearest_nodes(G, start_coords[1], start_coords[0])
    end_node = ox.distance.nearest_nodes(G, end_coords[1], end_coords[0])

    # Compute A* path with Euclidean heuristic
    route = nx.astar_path(G, start_node, end_node, 
                          heuristic=lambda u, v: euclidean_heuristic(u, v, G), 
                          weight='length')

    # Extract coordinates for the route
    route_coords = [(G.nodes[n]['y'], G.nodes[n]['x']) for n in route]

    # Create folium map
    m = folium.Map(location=route_coords[0], zoom_start=14)
    folium.PolyLine(route_coords, color="red", weight=5, opacity=0.8).add_to(m)
    folium.Marker(route_coords[0], popup="Start", icon=folium.Icon(color="green")).add_to(m)
    folium.Marker(route_coords[-1], popup="End", icon=folium.Icon(color="red")).add_to(m)

    return m, G, route

m, G, base_route = get_route_map(start_coords, end_coords)

st.write(f"Showing route from {start_coords} to {end_coords}")
st_data = st_folium(m, width=700, height=500)

# Route statistics
total_length = sum(ox.utils_graph.get_route_edge_attributes(G, base_route, 'length'))
st.success(f"Route Length: {total_length:.2f} meters")