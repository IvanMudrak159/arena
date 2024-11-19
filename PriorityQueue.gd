class_name PriorityQueueArray

var _data: Array = []  # Масив для зберігання пар [елемент, вартість]

func insert(element: Variant, cost: float) -> void:
    _data.append([element, cost])
    _data.sort_custom(sort_ascending)

func remove(element: Variant) -> void:
    for i in range(_data.size()):
        if _data[i][0] == element:
            _data.remove_at(i)
            break

func find(condition: Callable) -> Variant:
    for pair in _data:
        var element = pair[0]
        if condition.call(element):
            return element
    return null

func extract() -> Variant:
    if empty():
        return null
    return _data.pop_front()[0]

func empty() -> bool:
    return _data.is_empty()

func size() -> int:
    return _data.size()

func sort_ascending(a, b):
    if a[1] < b[1]:
        return true
    return false
