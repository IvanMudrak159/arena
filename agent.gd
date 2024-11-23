extends Node2D

@onready var ship = get_parent()
@onready var debug_path = ship.get_node('../debug_path')
@onready var debug_path_polygon = ship.get_node('../debug_path_polygon')
@onready var debug_path_target = ship.get_node('../debug_path_target')
@onready var debug_path_smooth = ship.get_node('../debug_path_smooth')
@onready var debug_path_portals = ship.get_node('../debug_path_portals')
@onready var debug_path_future = ship.get_node('../debug_path_future')
@onready var debug_path_nearestpoint = ship.get_node('../debug_path_nearestpoint')
@onready var debugText = ship.get_node('../debugText')
@onready var speedDebug = ship.get_node('../speedDebug')
@onready var angle1 = ship.get_node('../angle1')
@onready var angle2 = ship.get_node('../angle2')
@onready var angle3 = ship.get_node('../angle3')

var ticks = 0
var spin = 0
var thrust = false

func action(_walls: Array[PackedVector2Array], _gems: Array[Vector2], 
            _polygons: Array[PackedVector2Array], _neighbors: Array[Array]):

    ticks += 1
    markPolygons(_polygons)
    var gem = findNearestGemByDistance(ship.position, _gems)
    var polygonNodes: Array[PolygonNode] = setData(_polygons, _neighbors)
    var startNode = constructNode(ship.position, _polygons, _neighbors)
    var endNode = constructNode(gem, _polygons, _neighbors)

    if(startNode != null):
        markCurrentPolygon(ship.position, _polygons)
        var path = findPath(polygonNodes, startNode, endNode, true)
        
        var vectorPath : Array[Vector2] = []
        for polygonNode in path:
            vectorPath.append(polygonNode.center)
        vectorPath.append(gem)
        var steering = get_steering(vectorPath, ship, 150, 40, 10)

        #var smoothPath = getSmoothPath(path, ship.position, gem, _walls, false, false)
        #var steering = get_steering(smoothPath, ship, 150, 40, 10)
        return [steering[0], steering[1], false]
    #showNearestGemByDistance(ship.position, gem)
    spin = 1
    return [spin, true, false]

func get_steering(path: Array, ship: CharacterBody2D, max_speed: float, slowing_radius: float, path_radius: float) -> Array:
    var spin = 0
    var thrust = false
    
    if path.is_empty():
        spin = randi_range(-1, 1)
        thrust = true
        return [spin, thrust, false]
        
    path.remove_at(0)
    var target = path[0]

    var predicted_position = ship.position + ship.velocity.normalized() * 50
    debug_path_portals.clear_points()
    debug_path_portals.add_point(ship.position)
    debug_path_portals.add_point(ship.position + ship.velocity)

    
    var nearest_point_on_path = project_point_on_segment(ship.position, target, predicted_position)
    var distance_to_path = predicted_position.distance_to(nearest_point_on_path)
    debug_path_future.clear_points()
    debug_path_future.add_point(ship.position)
    debug_path_future.add_point(target)
    
    var direction_to_target = (target - ship.position).normalized()

    var velocityDirectionToTarget = direction_to_target - ship.velocity
    
    debug_path_nearestpoint.clear_points()
    debug_path_nearestpoint.add_point(ship.position + ship.velocity)
    debug_path_nearestpoint.add_point(target)

    
    var desired_velocity = direction_to_target * max_speed
    var angle_diff = wrapf(desired_velocity.angle() - ship.velocity.angle(), -PI, PI)
    var angle_diff_2 = wrapf(desired_velocity.angle() - ship.rotation, -PI, PI)
    var angle_diff_3 = wrapf(ship.velocity.angle() - ship.rotation, -PI, PI)
    
    speedDebug.text = str(ship.velocity.length())
    angle1.text = str(angle_diff)
    angle2.text = str(angle_diff_2)
    angle3.text = str(angle_diff_3)
    debugText.modulate = Color.GREEN
    
    
    if (ship.velocity.length() > 20):
        if(abs(angle_diff_2) < 0.1 && ship.velocity.length() < 70):
            spin = 0
            debugText.text = "Still little speed, bust to target"
            return [spin, isInSpeedLimit(spin, max_speed)]
    
        if(abs(angle_diff) < 0.1):
            if(abs(angle_diff_2) < 0.1):
                spin = 0
                debugText.text = "Normal speed, bust to target"
                return [spin, isInSpeedLimit(spin, max_speed)]
            else:
                debugText.text = "Rotate ship to target"
                spin = sign(angle_diff_2)
                return [spin, false]
        else:
            if(abs(angle_diff_3) > 2.3):
                debugText.text = "Change velocity to reduce difference with target"
                thrust = isInSpeedLimit(spin, max_speed)
            else:
                debugText.text = "Velocity and desired velocity too different, rotate to stop"
                spin = sign(angle_diff)
                
    else:
        if(abs(angle_diff_2) < 0.1):
            spin = 0
            debugText.text = "Little speed, bust to target"
            return [spin, isInSpeedLimit(spin, max_speed)]
        else:
            debugText.text = "Rotate ship to target"
            spin = sign(angle_diff_2)
    
    var newVelocity = ship.velocity + Vector2.from_angle(ship.rotation + spin * 3) * 4
    var thrustLimit : bool = ship.velocity.length() < max_speed || newVelocity.length() < ship.velocity.length()

    
    #if distance_to_path < path_radius:
        #debug_path_future.default_color = Color(0, 1, 0)
        #if(abs(angle_diff_2) > 0.3):
            #spin = sign(abs(angle_diff_2))
        #else:
            #thrust = true
    #else:
        #debug_path_future.default_color = Color(1, 0, 0)
        #if (ship.velocity.length() > 20):
            #if(abs(angle_diff_2) < 0.3 && ship.velocity.length() < 70):
                #spin = 0
                #return [spin, true]
#
            #if(abs(angle_diff) < 0.2):
                #spin = 0
                #return [spin, true]
            #spin = sign(abs(angle_diff))
            #if(abs(angle_diff_3) > 2):
                #thrust = true
        #else:
            #if(abs(angle_diff_2) < 0.1):
                #spin = 0
                #return [spin, true]
            #else:
                #spin = sign(angle_diff_2)
    
    return [spin, thrust]
    
func isInSpeedLimit(spin: int, max_speed: float) -> bool:
    var newVelocity = ship.velocity + Vector2.from_angle(ship.rotation + spin * 3) * 4
    var thrustLimit : bool = ship.velocity.length() < max_speed || newVelocity.length() < ship.velocity.length()
    return thrustLimit
    
func project_point_on_segment(P: Vector2, A: Vector2, B: Vector2) -> Vector2:
    var AB = B - A
    var AP = P - A
    var t = AP.dot(AB) / AB.length_squared()
    t = clamp(t, 0, 1)
    return A + AB * t

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

func getSmoothPath(path: Array[PolygonNode], start: Vector2, end: Vector2, walls: Array[PackedVector2Array], showPortals: bool, showSmoothPath: bool):
    var vectorPath: Array[PackedVector2Array] = getVectorPath(path)
    var packedEnd: PackedVector2Array = PackedVector2Array([end])
    vectorPath.append(packedEnd)
    
    var portals: Array[PackedVector2Array] = getPortals(vectorPath)
    if(showPortals): showPortals(start, end, portals)
    #var funnel : Array[Vector2] = funnel_path(portals, start, end)
    var funnel : Array[Vector2] = navigation_corridor_path(portals, start, end)
    if(showSmoothPath): showSmoothPath(funnel)
    return funnel

func getPortals(path: Array[PackedVector2Array]) -> Array[PackedVector2Array]:
    var shared_edges: Array[PackedVector2Array] = []
    for i in range(path.size() - 1):
        var polygon1 = path[i]
        var polygon2 = path[i + 1]
        
        var shared_edge = find_shared_edge(polygon1, polygon2)
        shared_edge.reverse()
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

func navigation_corridor_path(portals: Array[PackedVector2Array], start: Vector2, end: Vector2) -> Array[Vector2]:
    var path: Array[Vector2] = [start]
    var current_pos = end

    for portal in portals:
        var left = portal[0]
        var right = portal[1]

        # Calculate which portal edge leads closer to the target
        var target = right
        if current_pos.distance_to(left) < current_pos.distance_to(right):
            target = left

        # Add the target point to the path
        path.append(target)
        current_pos = target

    # Append the final endpoint
    if path[-1] != end:
        path.append(end)
    return path

func optimize_path(path: Array[PackedVector2Array], walls: Array[PackedVector2Array]) -> Array[Vector2]:
    if(path.size() == 0): return []
    var optimized_path: Array[Vector2]
    var current_point = path[0][0]

    for i in range(1, path.size()):
        if is_path_clear(current_point, path[i][0], walls):
            # Пропускаємо проміжну точку, якщо шлях прямий
            continue
        else:
            # Додаємо останню допустиму точку перед перешкодою
            optimized_path.append(path[i - 1][0])
            current_point = path[i - 1][0]
    
    # Додаємо останню точку шляху
    optimized_path.append(path[-1][0])
    return optimized_path

func is_path_clear(start: Vector2, end: Vector2, walls: Array[PackedVector2Array]) -> bool:
    for wall in walls:
        for i in range(wall.size()):
            var p1 = wall[i]
            var p2 = wall[(i + 1) % wall.size()]  # Наступна вершина (циклічно)
            if segments_intersect(start, end, p1, p2):
                return false
    return true

func segments_intersect(p1: Vector2, p2: Vector2, q1: Vector2, q2: Vector2) -> bool:
    var o1 = orientation(p1, p2, q1)
    var o2 = orientation(p1, p2, q2)
    var o3 = orientation(q1, q2, p1)
    var o4 = orientation(q1, q2, p2)

    if o1 != o2 and o3 != o4:
        return true  # Загальний випадок

    # Перевірка спеціальних випадків
    if o1 == 0 and on_segment(p1, q1, p2): return true
    if o2 == 0 and on_segment(p1, q2, p2): return true
    if o3 == 0 and on_segment(q1, p1, q2): return true
    if o4 == 0 and on_segment(q1, p2, q2): return true
    return false

func orientation(a: Vector2, b: Vector2, c: Vector2) -> int:
    var val = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y)
    if val == 0:
        return 0  # Колінеарні
    return 1 if val > 0 else -1  # 1 - за годинниковою стрілкою, -1 - проти

func on_segment(a: Vector2, b: Vector2, c: Vector2) -> bool:
    return min(a.x, c.x) <= b.x <= max(a.x, c.x) and min(a.y, c.y) <= b.y <= max(a.y, c.y)

func funnel_path(portals: Array[PackedVector2Array], start: Vector2, end: Vector2) -> Array[Vector2]:
    var path: Array[Vector2] = [start]
    var apex: Vector2 = start
    var apexIndex = 0
    var leftIndex = 0
    var rightIndex = 0
    if portals.size() == 0:
        path.append(end)
        return path
    var left_apex = portals[0][0]
    var right_apex = portals[0][1]
    for i in range(1, portals.size()):
        var left = portals[i][0]
        var right = portals[i][1]
        
        if triangle_area2(apex, left_apex, left) > 0:
            if apex == left_apex or triangle_area2(apex, right_apex, left) < 0:
                left_apex = left
                leftIndex = i
            else:
                path.append(right_apex)
                apex = right_apex
                apexIndex = rightIndex
                left_apex = apex
                right_apex = apex
                i = rightIndex
                leftIndex = apexIndex;
                rightIndex = apexIndex; 
                continue
                
        if triangle_area2(apex, right_apex, right) < 0:
            if apex == right_apex or triangle_area2(apex, left_apex, right) > 0:
                right_apex = right
                rightIndex = i
            else:
                path.append(left_apex)
                apex = left_apex
                apexIndex = leftIndex
                left_apex = apex
                right_apex = apex
                i = leftIndex
                leftIndex = apexIndex;
                rightIndex = apexIndex; 
                continue
        
    if apex != end:
        path.append(end)
    return path

func triangle_area2(a: Vector2, b: Vector2, c: Vector2) -> float:
    var value = -1 * ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x))
    return value

func reconstruct_path(node: AStarNode) -> Array[PolygonNode]:
    var path: Array[PolygonNode] = []
    while node != null:
        path.insert(0, node.polygonNode)
        node = node.parent
    return path

func bounce():
    return

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

func showNearestGemByDistance(shipPosition: Vector2, gem: Vector2):
    debug_path_target.clear_points()
    debug_path_target.add_point(ship.position)
    debug_path_target.add_point(gem)

func is_left(p0: Vector2, p1: Vector2, p2: Vector2) -> float:
    return (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y)
