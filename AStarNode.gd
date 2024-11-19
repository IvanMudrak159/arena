class_name AStarNode
extends Object

var polygonNode: PolygonNode
var g_cost: float = 0  # Вартість від початкового вузла
var h_cost: float = 0  # Евристична вартість до цільового вузла
var parent: AStarNode = null

func _init(polygonNode: PolygonNode, g_cost: float, h_cost: float, parent: AStarNode) -> void:
    self.polygonNode = polygonNode
    self.g_cost = g_cost
    self.h_cost = h_cost
    self.parent = parent
    
func f_cost() -> float:
    return g_cost + h_cost

func _to_string() -> String:
    return "AStarNode(polygonNode=%s, g_cost=%.2f, h_cost=%.2f, f_cost=%.2f, parent=%s)" % [
        str(polygonNode),
        g_cost,
        h_cost,
        f_cost(),
        parent != null and str(parent.polygonNode) or "null"
            ]
