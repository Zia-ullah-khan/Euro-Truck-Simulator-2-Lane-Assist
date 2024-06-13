extends Node

@export var url = "http://127.0.0.1:37520/api/tags/data"
@export var tag = "map"
@export var updateRate = 20 # Seconds
@onready var HTTPRequestObject = $/root/Node3D/HTTPRequest

var MapData = null
var lastUpdateTime = null

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.

func send_request() -> void:
	var headers = ["Content-Type: application/json"]
	var json = JSON.stringify({
		"tag": tag
	})
	HTTPRequestObject.request_completed.connect(parse_request)
	print(json)
	HTTPRequestObject.request(url, headers, HTTPClient.METHOD_POST, json)
	print("Request map data...")

func parse_request(result, response_code, headers, body):
	var json = JSON.parse_string(body.get_string_from_utf8())
	MapData = json

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	if lastUpdateTime == null:
		send_request()
		lastUpdateTime = Time.get_ticks_msec()
	if Time.get_ticks_msec() - lastUpdateTime > updateRate * 1000:
		send_request()
		lastUpdateTime = Time.get_ticks_msec()
