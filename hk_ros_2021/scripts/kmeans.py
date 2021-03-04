import random
import numpy as np
import math
import copy

#returns the distance between two points [x1,y1] and [x2,y2]
def dist_between(p1,p2):
    return math.sqrt( pow(p1[0] - p2[0],2) + pow(p1[1] - p2[1],2) )

#returns the centroids of the clusters and the separate clusters
#nk is the amount of clusters and data is an array of points, a point is [x,y]
def k_means(nk, data):
    k_moved = [0 for _ in range(nk)]

    clusters = [[] for _ in range(nk)]

    ds = np.array(data)
    np.seterr(divide='ignore', invalid='ignore')
    centroids = [ds[0]]
    k = []

    """ for _ in range(1, nk):
        dist_sq = np.array([min([np.inner(c-x,c-x) for c in centroids]) for x in ds])
        print(data)
        probs = dist_sq/dist_sq.sum()
        cumulative_probs = probs.cumsum()
        r = np.random.rand()
        
        for j, p in enumerate(cumulative_probs):
            if r < p:
                i = j
                break
        
        centroids.append(ds[i]) """
    for _ in range(nk):
        k.append(random.choice(data))

    #k = centroids
    k_old = copy.deepcopy(k)

    bigMove = 2
    while bigMove > 0.1:
        clusters = [[] for _ in range(nk)]
        for point in data:
            distance = [0 for _ in range(nk)]
            for i in range(nk):
                distance[i] = math.sqrt( pow(point[0] - k[i][0],2) + pow(point[1] - k[i][1],2))
            clusters[distance.index(min(distance))].append(point)
        for i in range(nk):
            if len(clusters[i]) > 0:
                cluster_average = [sum(x)/len(x) for x in zip(*clusters[i])]
                k[i] = cluster_average
                k_moved[i] = math.sqrt( pow(k[i][0] - k_old[i][0],2) + pow(k[i][1] - k_old[i][1],2) )
                k_old[i] = copy.deepcopy(k[i])
            else:
                k_moved[i] = 0
        
        bigMove = max(k_moved)

    return k,clusters

#takes an array of clusters to determine the silhouette coefficient
def silhouette_coefficient(clusters):
    s_val_local = []
    b_min = 0
    a_val = 0
    for clus_m in clusters:
        for p_main in clus_m:
            a_total = 0
            b_total = 0
            b_count = 0
            for p_c in clus_m:
                if p_main == p_c:
                    continue
                a_total = a_total + dist_between(p_main,p_c)
            if len(clus_m) > 1:
                a_val = (a_total/ (len(clus_m)-1) )
            else:
                a_val = 1

            b_values = []
            for clus_c in clusters:
                b_count = 0
                b_total = 0
                if clus_m == clus_c:
                    continue
                for p_c in clus_c:
                    b_count = b_count + 1
                    b_total = b_total + dist_between(p_main,p_c)
                if b_count == 0:
                    b_values.append(1)
                else:
                    b_values.append(b_total/b_count)
            b_min = min(b_values)

            s_val_local.append( (b_min - a_val)/max([a_val,b_min]) )
    return sum(s_val_local)/len(s_val_local)

#moves all points towards the closest highest density
def concentrate_points(points,distance,delta,iterations):
    for _ in range(iterations):
        new_points = []
        for p1 in points:
            closest_points = []
            for p2 in points:
                if p1 == p2:
                    continue
                if dist_between(p1,p2) <= distance:
                    closest_points.append(p2)
            if len(closest_points) > 0:
                avg = [sum(x)/len(x) for x in zip(*closest_points)]
                new_points.append([p1[0] + (avg[0]-p1[0])*delta,p1[1] + (avg[1]-p1[1])*delta])
            else:
                new_points.append(p1)
        points = copy.deepcopy(new_points)
    return new_points

#returns the most likely amount of clusters for the given data
def most_likely_k(k_tests,data):
    s_values = []
    for nk in k_tests:
        _,clusters = k_means(nk,data)
        s_values.append(silhouette_coefficient(clusters))
    return k_tests[s_values.index(max(s_values))]

#ppc = points per cluster
def generate_cluster(n_min,n_max,size,deviation,ppc):
    n_clusters = random.randint(n_min,n_max)
    source = []
    for _ in range(n_clusters):
        source.append([random.randint(0,size),random.randint(0,size)])
    points = []
    for s in source:
        for _ in range(ppc):
            points.append( [s[0] + random.random()*deviation-deviation/2, s[1] + random.random()*deviation-deviation/2] )
    return n_clusters,source,points