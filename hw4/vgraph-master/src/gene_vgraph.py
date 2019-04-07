"""
This part contains two steps:
a) Generate all edges:
	a.1) All obstacle edges
	a.2) Edges connecting start -> vertices
	a.3) Edges connecting goal -> vertices
	a.4) Edges connecting vertices from different obstacles
b) Use collision checker (intersection script) to remove unwanted edges:
	for i in all_edges:
		for j in obstacle_edges:
			if i==j or i==j[-1]: 
				break # keep all obstacle edges
			elif i and j have common points: 
				continue
			else:
				if intersect:
					remove collisions
"""

def ccw(A,B,C):
	return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])

def intersect(A,B,C,D):
	return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

