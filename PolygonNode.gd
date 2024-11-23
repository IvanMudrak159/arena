class_name PolygonNode
extends Object

var polygon: PackedVector2Array
var index: int
var center: Vector2
var neighbourIndexes: Array

func _init(polygon: PackedVector2Array, index: int, center: Vector2, neighbourIndexes: Array) -> void:
    self.polygon = polygon
    self.index = index
    self.center = center
    self.neighbourIndexes = neighbourIndexes

func _to_string() -> String:
    return "PolygonNode(center=%s, neighbourIndexes=%s)" % [
        str(center), str(neighbourIndexes)
    ]

func equals(other: PolygonNode) -> bool:
    if(other == null): return false
    return self.center == other.center and self.polygon == other.polygon
