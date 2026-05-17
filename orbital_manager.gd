@tool

extends Node
# Constante Gravitacional Universal G en m3 kg−1 s−2

const G: float = 0.000000000066743

const MIN_ATTRACTOR_MASS:float = 1e7

const LIMIT_TIME_SCALE = 8

var _time_added: float = 0
var _time_scale: float = 1
var _t_change: float

func get_time_scale() -> float:
	return _time_scale

func set_time_scale(v: float, limited:bool = false) -> void:
	if limited and v > LIMIT_TIME_SCALE:return
	_t_change = get_current_time()
	_time_added = _t_change - (_t_change - _time_added) * v / _time_scale
	_time_scale = v
	print("Time scale: ", v)

func get_current_time() -> float:
	return Time.get_ticks_msec() / 1000.0 * _time_scale + _time_added

func limit_time_scale() -> void:
	if _time_scale > LIMIT_TIME_SCALE: set_time_scale(LIMIT_TIME_SCALE)
