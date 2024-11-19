extends Node2D

@onready var ship = get_parent()
@onready var debug_path = ship.get_node('../debug_path')
@onready var debug_path_polygon = ship.get_node('../debug_path_polygon')
@onready var debug_path_target = ship.get_node('../debug_path_target')
@onready var debug_path_smooth = ship.get_node('../debug_path_smooth')
@onready var debug_path_portals = ship.get_node('../debug_path_portals')

var ticks = 0
var spin = 0
var thrust = false

func action(_walls: Array[PackedVector2Array], _gems: Array[Vector2], 
            _polygons: Array[PackedVector2Array], _neighbors: Array[Array]):

    ticks += 1
    if ticks % 30 == 1:
        spin = randi_range(-1, 1)
        thrust = bool(randi_range(0, 1))
        return [spin, thrust, false]
        
    markPolygons(_polygons)
    
    var gem = findNearestGemByDistance(ship.position, _gems)
    var polygonNodes: Array[PolygonNode] = setData(_polygons, _neighbors)
    var startNode = constructNode(ship.position, _polygons, _neighbors)
    var endNode = constructNode(gem, _polygons, _neighbors)

    if(startNode != null):
        markCurrentPolygon(ship.position, _polygons)
        var path = findPath(polygonNodes, startNode, endNode, false)
        var smoothPath = getSmoothPath(path, ship.position, gem, false, true)
    
    debug_path_target.clear_points()
    debug_path_target.add_point(ship.position)
    debug_path_target.add_point(gem)
    return [spin, thrust, false]

func setData(_polygons: Array[PackedVector2Array], _neighbors: Array[Array]) -> Array[PolygonNode]:
    var polygonNodes: Array[PolygonNode] = []
    if(_polygons.size() != _neighbors.size()): return polygonNodes
    
    for i in range(_polygons.size()):
        var polygon = _polygons[i]
        var center = getCenter(polygon)
        var neighbors = _neighbors[i]
        var polygonNode = PolygonNode.new(polygon, i, center, neighbors)
        polygonNodes.append(polygonNode)
    return polygonNodes

func constructNode(position: Vector2, _polygons: Array[PackedVector2Array], _neighbors: Array[Array]) -> PolygonNode:
    var i = getContainerPolygonId(position, _polygons)
    if(i == null): return null
    var polygon = _polygons[i]
    var center = getCenter(polygon)
    var neighbors = _neighbors[i]
    var polygonNode = PolygonNode.new(polygon, i, center, neighbors)
    return polygonNode

func findPath(polygonData: Array[PolygonNode], startPolygon: PolygonNode, endPolygon: PolygonNode, showPath: bool) -> Array[PolygonNode]:
    var path = aStar(polygonData, startPolygon, endPolygon)
    if(showPath):
        showPath(path)
    return path

func getSmoothPath(path: Array[PolygonNode], start: Vector2, end: Vector2, showPortals: bool, showSmoothPath: bool):
    if(path.size() == 1): return []
    var portals: Array[PackedVector2Array] = getPortals(getVectorPath(path))
    if(showPortals):
        showPortals(start, end, portals)
    var funnel = funnel_path(portals, start, end)
    if(showSmoothPath):
        showSmoothPath(funnel)
    return funnel

func getPortals(path: Array[PackedVector2Array]) -> Array[PackedVector2Array]:
    var shared_edges: Array[PackedVector2Array] = []
    for i in range(path.size() - 1):
        var polygon1 = path[i]
        var polygon2 = path[i + 1]
        
        var shared_edge = find_shared_edge(polygon1, polygon2)
        if shared_edge.size() == 2:
            shared_edges.append(shared_edge)
        else:
            push_error("No shared edge found between polygons at index %d and %d" % [i, i + 1])
    
    return shared_edges

func find_shared_edge(polygon1: PackedVector2Array, polygon2: PackedVector2Array) -> PackedVector2Array:
    var shared_vertices = []
    for vertex1 in polygon1:
        for vertex2 in polygon2:
            if vertex1 == vertex2:
                shared_vertices.append(vertex1)
    
    if shared_vertices.size() == 2:
        return PackedVector2Array(shared_vertices)
    else:
        return PackedVector2Array()

func funnel_path(portals: Array[PackedVector2Array], start: Vector2, end: Vector2) -> Array[Vector2]:
    var path: Array[Vector2] = [start]  # Final smoothed path
    var apex: Vector2 = start
    var left_index = 0
    var right_index = 0
    
    var left_apex = portals[0][0]
    var right_apex = portals[0][1]

    for i in range(1, portals.size()):
        var left = portals[i][0]
        var right = portals[i][1]

        # Check for a new right portal
        if triangle_area2(apex, right_apex, right) <= 0:
            if apex == right_apex or triangle_area2(apex, left_apex, right) > 0:
                # Tighten the funnel on the right
                right_apex = right
                right_index = i
            else:
                # Apex to left, new path segment
                path.append(left_apex)
                apex = left_apex
                left_apex = left
                right_apex = right
                left_index = i
                right_index = i
                continue

        # Check for a new left portal
        if triangle_area2(apex, left_apex, left) >= 0:
            if apex == left_apex or triangle_area2(apex, right_apex, left) < 0:
                # Tighten the funnel on the left
                left_apex = left
                left_index = i
            else:
                # Apex to right, new path segment
                path.append(right_apex)
                apex = right_apex
                left_apex = left
                right_apex = right
                left_index = i
                right_index = i
                continue
    
    # Add the final point to the path
    path.append(end)
    return path

func triangle_area2(a: Vector2, b: Vector2, c: Vector2) -> float:
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)

func reconstruct_path(node: AStarNode) -> Array[PolygonNode]:
    var path: Array[PolygonNode] = []
    while node != null:
        path.insert(0, node.polygonNode)
        node = node.parent
    return path

# Called every time the agent has bounced off a wall.
func bounce():
    return

# Called every time a gem has been collected.
func gem_collected():
    return
    
### PATHFINDING METHODS
func aStar(polygonData: Array[PolygonNode], startPolygon: PolygonNode, endPolygon: PolygonNode) -> Array[PolygonNode]:
    var open_set = PriorityQueueArray.new()
    var closed_set = []  # Закриті вузли
    
    var start_node = AStarNode.new(startPolygon, 0, 0, null)
    open_set.insert(start_node, start_node.f_cost())
    while open_set.size() > 0:
        var current_node = open_set.extract()
        
        if current_node.polygonNode.equals(endPolygon):
            return reconstruct_path(current_node)
        
        closed_set.append(current_node.polygonNode.polygon)
        
        for neighbor_index in current_node.polygonNode.neighbourIndexes:
            var neighborPolygonNode = polygonData[neighbor_index]
            if closed_set.has(neighborPolygonNode.polygon):
                continue
            
            var neighbor_node = AStarNode.new(
                neighborPolygonNode,
                current_node.g_cost + current_node.polygonNode.center.distance_to(neighborPolygonNode.center),
                neighborPolygonNode.center.distance_to(endPolygon.center),
                current_node)
            
            var condition = func(n): return n.polygonNode.center == neighbor_node.polygonNode.center
            
            var existing_node = open_set.find(condition)
            if existing_node == null or neighbor_node.g_cost < existing_node.g_cost:
                open_set.remove(existing_node)
                open_set.insert(neighbor_node, neighbor_node.f_cost())
    return []
    
### HELPER METHODS

func getContainerPolygon(position: Vector2, _polygons):
    for polygon in _polygons:
        if(is_point_in_polygon_winding(position, polygon)): return polygon

func getContainerPolygonId(position: Vector2, _polygons):
    for i in range(_polygons.size()):
        if(is_point_in_polygon_winding(position, _polygons[i])): return i

func getCenter(polygon: PackedVector2Array) -> Vector2:
    var centerX = 0
    var centerY = 0
    for point in polygon:
        centerX += point.x
        centerY += point.y
    var centerPoint = Vector2(centerX / polygon.size(), centerY / polygon.size())
    return centerPoint

func findNearestGemByDistance(playerPosition: Vector2, _gems: Array[Vector2]) -> Vector2:
    var minDistance: float = 10000000000000000 
    var nearestGem
    for gem in _gems:
        var distance = playerPosition.distance_squared_to(gem)
        if(distance < minDistance):
            minDistance = distance
            nearestGem = gem
    return nearestGem

### DEBUG METHODS

func printPolygonsInfo(_polygons: Array[PackedVector2Array]):
    if ticks % 10000000000000000 == 1:
        for polygon in _polygons:
            print(polygon)

func printNeighborsInfo(_neighbors: Array[Array]):
    if ticks % 10000000000000000 == 1:
        for neighbor in _neighbors:
            print(neighbor)

func markPolygons(_polygons: Array[PackedVector2Array]):
    if ticks % 10000000000000000 != 1: return
    for i in range(_polygons.size()):
        var center = getCenter(_polygons[i])
        var label = Label.new()
        label.text = str(i)
        label.position = center
        get_tree().root.get_child(0).add_child(label)

func markCurrentPolygon(_playerPos, _polygons: Array[PackedVector2Array]):
    var polygon = getContainerPolygon(_playerPos, _polygons)
    debug_path_polygon.clear_points()
    for point in polygon:
        debug_path_polygon.add_point(point)
    debug_path_polygon.add_point(polygon[0])
    return

func showPath(path: Array) -> void:
    debug_path.clear_points()
    if path.size() != 0:
        for polygon in path:
            debug_path.add_point(polygon.center)

func showSmoothPath(path: Array) -> void:
    debug_path_smooth.clear_points()
    if path.size() != 0:
        for polygon in path:
            debug_path_smooth.add_point(polygon)

func showPortals(start: Vector2, end: Vector2, portals: Array[PackedVector2Array]) -> void:
    debug_path_portals.clear_points()
    debug_path_portals.add_point(start)
    if portals.size() != 0:
        for portal in portals:
            debug_path_portals.add_point(getCenterPoint(portal))
        debug_path_portals.add_point(end)

func getCenterPoint(points: PackedVector2Array) -> Vector2:
    var x = 0
    var y = 0
    for point in points:
        x += point.x
        y += point.y
    return Vector2(x / points.size(), y / points.size())

func getVectorPath(path: Array[PolygonNode]) -> Array[PackedVector2Array]:
    var vectorPath : Array[PackedVector2Array] = []
    for polygonNode in path:
        vectorPath.append(polygonNode.polygon)
    return vectorPath

func is_point_in_polygon_winding(point: Vector2, polygon: PackedVector2Array) -> bool:
    var winding_number = 0
    var vertex_count = polygon.size()

    for i in range(vertex_count):
        var current_vertex = polygon[i]
        var next_vertex = polygon[(i + 1) % vertex_count]

        if current_vertex.y <= point.y:
            if next_vertex.y > point.y and is_left(current_vertex, next_vertex, point) > 0:
                winding_number += 1
        else:
            if next_vertex.y <= point.y and is_left(current_vertex, next_vertex, point) < 0:
                winding_number -= 1

    return winding_number != 0

func is_left(p0: Vector2, p1: Vector2, p2: Vector2) -> float:
    return (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y)
