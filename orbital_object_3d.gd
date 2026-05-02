@tool

## Keplerian Orbital Simulation Node
##
## OrbitalObject3D extends Node3D and serves as the core component for gravity-based orbital simulation.[br]
## It implements the Patched Conics Approximation within a hierarchical reference frame. Orbital propagation is performed in the local coordinate space (transform) of the current attractor using the Classic Orbital Elements (COEs).[br]
## Position updates occur every _process(delta) via analytical solutions (Kepler’s Equation solved with Newton-Raphson for elliptical orbits) and numerical fallbacks for singular cases (radial or parabolic motion).[br]
## Attractor switching is handled automatically through gravitational Sphere of Influence (SOI) validation.[br][br]
## Important: this model deliberately ignores perturbations from bodies other than the current attractor, preserving the simplicity and performance of the restricted two-body problem at every time step.
 
class_name OrbitalObject3D
extends Node3D

## Emitted whenever the orbital elements or trajectory of this body are updated.
signal orbit_changed()
## Emitted when a new satellite/orbiter enters this body's Sphere of Influence and becomes its child.
signal orbiter_added(orbiter:OrbitalObject3D)
## Emitted when a satellite/orbiter leaves this body's Sphere of Influence and is no longer its child.
signal orbiter_removed(orbiter:OrbitalObject3D)
## Emitted when this body changes its current gravitational attractor (parent body).
signal has_new_attractor(attractor:OrbitalObject3D)

# --- Propiedades Exportables ---
## The gravitational mass of this body, used in all orbital calculations (e.g., specific orbital energy, acceleration, and time period).
@export var mass: float = 1.0
@export var radius: float = 0.5

#var vel_basis:Basis
#@export_group("Initial position espherical", "pos_")
#@export var pos_radius: float = 0.0:
	#set(v):
		#pos_radius = maxf(0.0, v)
		#_update_position()
#@export_range(0.0,360.0,0.1,"º") var pos_theta: float = 0.0:
	#set(v):
		#pos_theta = v
		#_update_position()
#@export_range(-90,90,0.1,"º") var pos_phi: float = 0.0:
	#set(v):
		#pos_phi = v
		#_update_position()
#func _update_position():
	#var x:float = pos_radius * cos(deg_to_rad(pos_phi)) * cos(deg_to_rad(pos_theta))
	#var y:float = pos_radius * cos(deg_to_rad(pos_phi)) * sin(deg_to_rad(pos_theta))
	#var z:float = pos_radius * sin(deg_to_rad(pos_phi))
	#_initial_position = Vector3(x, y, z)
	#var i = Vector3(x, y, z).normalized()
	#var j = -i.cross(Vector3(0,0,1)).normalized()
	#var k = i.cross(j).normalized()
	#vel_basis = Basis(i, j, k)
	#_update_velocity()
#
#@export_group("Initial velocity espherical", "vel_")
#@export var vel_speed:float = 0.0:
	#set(v):
		#vel_speed = v
		#_update_velocity()
#@export_range(-90,90,0.1,"º") var vel_elevation:float = 0.0:
	#set(v):
		#vel_elevation = v
		#_update_velocity()
#@export_range(-90,90,0.1,"º") var vel_azimuth:float = 0.0:
	#set(v):
		#vel_azimuth = v
		#_update_velocity()
#func _update_velocity():
	#var x = vel_speed * cos(deg_to_rad(vel_azimuth)) * sin(deg_to_rad(vel_elevation))
	#var y = vel_speed * cos(deg_to_rad(vel_azimuth)) * cos(deg_to_rad(vel_elevation))
	#var z = vel_speed * sin(deg_to_rad(vel_azimuth))
	#_initial_velocity = vel_basis * Vector3(x, y, z)

@export var _initial_position: Vector3= Vector3.ZERO: set = _set_initial_position
@export var _initial_velocity: Vector3 = Vector3.ZERO: set = _set_initial_velocity
## Show or hide the node's log messages
@export var _show_log_msgs: bool = true
## Use to load references to another node; not used in orbital calculations.
@export var related_to: Node
## The classification of the current trajectory determined by the calculated eccentricity e and specific energy. This is an enum value (ORBITS_TYPES) indicating whether the orbit is radial, elliptic, parabolic, or hyperbolic.
var orbit_type: ORBITS_TYPES = ORBITS_TYPES.INDETERMINATE
var _initial_time:float
func _set_initial_position(v:Vector3):
	_initial_position = v
	if Engine.is_editor_hint():
		position = _initial_position
		orbit_type = ORBITS_TYPES.INDETERMINATE
func _set_initial_velocity(v:Vector3):
	_initial_velocity = v
	if Engine.is_editor_hint(): orbit_type = ORBITS_TYPES.INDETERMINATE

##This enumeration defines the possible classifications of a body's trajectory, determined by its eccentricity and specific orbital energy relative to its current attractor.
enum ORBITS_TYPES {
	INDETERMINATE, ## Initial state or temporary state where the orbital elements have not yet been successfully calculated or classified. This state forces an immediate call to the orbit calculation
	RADIAL,
	ELIPTIC,
	PARABOLIC,
	HYPERBOLIC,
	CENTRAL_OBJECT ## State assigned to a body that has no attractor (i.e., the root of the orbital hierarchy). Indicates the body is considered stationary in its own frame of reference.
}

## An array containing references to all OrbitalBody3D nodes currently using this body as their gravitational attractor. This list is actively managed during sphere of influence (SOI) transfers.
var orbiters: Array[Node]:
	get:
		return _orbiters.duplicate(true)
var _orbiters: Array[Node]
var massive_orbiters:Array[Node]:
	get:
		return _massive_orbiters.duplicate(true)
var _massive_orbiters:Array[Node]
## It takes the value true when the mass is greater than MIN_ATTRACTOR_MASS of the OrbitalManager singleton, indicating that the object is considered a possible attractor and its radius of influence is calculated to detect incoming and outgoing objects.
var is_massive:bool
## The calculated radius (in world units) of this body's Sphere of Influence (SOI) relative to its own attractor. Orbiters will use this value to detect when to transition their frame of reference to this body.
var influence_radius: float = 0.0
## The current central gravitational body that this node is orbiting. All position and velocity state vectors of this body are calculated relative to the attractor's local coordinate system.
var attractor: OrbitalObject3D # Referencia al cuerpo central

var _perifocal_transform:Transform3D
var _last_transform: Transform3D
var _temp_velocity: Vector3 = Vector3.ZERO

# ====================================================================

func _ready() -> void:
	# Asegurar que el autoload existe
	if not is_instance_valid(OrbitalManager):
		push_error("El Autoload 'OrbitalManager' no está configurado.")
		return
	if mass > OrbitalManager.MIN_ATTRACTOR_MASS: is_massive = true
	else: is_massive = false
	var children: Array[Node] = get_children()
	for child in children:
		if not child is OrbitalObject3D: continue
		var orb_pos:Vector3 = child._initial_position
		var orb_vel:Vector3 = child._initial_velocity
		add_orbiter(child, orb_pos, orb_vel)
	
	set_notify_transform(true)
	_last_transform = global_transform

func _notification(what: int) -> void:
	if what == NOTIFICATION_TRANSFORM_CHANGED and Engine.is_editor_hint():
		if _last_transform != global_transform:
			orbit_type = ORBITS_TYPES.INDETERMINATE
			_initial_position = position
			_last_transform = global_transform

func _process(_delta: float) -> void:
	if Engine.is_editor_hint():
		if not attractor and get_parent() is OrbitalObject3D:
			attractor = get_parent()
		if not orbit_type:
			orbit_type = calcule_orbit(_initial_position, _initial_velocity)
			position = _initial_position
		return
	
	if not orbit_type:
		orbit_type = calcule_orbit(_initial_position, _initial_velocity)
	if orbit_type != ORBITS_TYPES.INDETERMINATE:
		_chek_orbiters()
		if orbit_type != ORBITS_TYPES.CENTRAL_OBJECT:
			var current_time = OrbitalManager.get_current_time() - _initial_time
			# Obtener posición
			position = _get_position_at_time(current_time)

# ==================================    ================================== #

func _get_position_at_time(t: float) -> Vector3:
	if not orbit_type:
		return position
	
	var pos_perifocal: Vector3
	
	var a = semi_major_axis
	var e = eccentricity
	var mu = mu_attractor
	var n = sqrt( mu / abs(pow(a, 3)))
	# 1. Calcular la Anomalía Media (M)
	var mean_anomaly = _mean_anomaly_at_t0 + n * (t)
	
	match orbit_type:
		ORBITS_TYPES.RADIAL:
			# La dirección es constante (dada por la posición inicial)
			var r_unit: Vector3 = _initial_position.normalized()
			
			# Necesitas una función que resuelva el movimiento radial unidimensional
			var result = _solve_radial_motion(t, _initial_position.length(), _initial_velocity.dot(r_unit), mu)
			
			var pos_magnitude = result.position_magnitude
			
			#La posición final es solo la magnitud a lo largo de la dirección unitaria
			pos_perifocal = r_unit * pos_magnitude
	
		ORBITS_TYPES.ELIPTIC:
			mean_anomaly = fmod(mean_anomaly, 2.0 * PI)
			if mean_anomaly < 0.0: mean_anomaly += 2.0 * PI
			
			# 2. M -> E (Anomalía Excéntrica)
			var E = _solve_kepler_newton_raphson(mean_anomaly, e)
			
			# 3. Posición (r_vec) en el Plano Perifocal (Xp, Yp, Zp=0)
			var xp = a * (cos(E) - e)
			var yp = a * sqrt(1.0 - e*e) * sin(E)
			pos_perifocal = Vector3(xp, yp, 0.0) # Plano orbital XY
		
		ORBITS_TYPES.HYPERBOLIC:
			# 2. M -> F (Anomalía Excéntrica Hiperbólica)
			var F = _solve_kepler_hyperbolic(mean_anomaly, e) # ¡CORRECCIÓN APLICADA!
			
			# Nota: En hiperbólica, 'a' debe ser usado con valor absoluto
			# Aunque lo calculamos como positivo, aseguramos la definición
			var abs_a = abs(a) 
			
			# Posición Hiperbólica
			# r = a(e*cosh(F) - 1). La definición de xp e yp es la correcta
			var xp = abs_a * (e - cosh(F))
			var yp = abs_a * sqrt(e*e - 1.0) * sinh(F)
			pos_perifocal = Vector3(xp, yp, 0.0)
		
		ORBITS_TYPES.PARABOLIC:
			return position
	return _perifocal_transform.basis * pos_perifocal

func _get_velocity_at_time(t: float) -> Vector3:
	if not orbit_type:
		return _temp_velocity
	
	var vel_perifocal: Vector3
	
	var a = semi_major_axis
	var e = eccentricity
	var mu = mu_attractor
	var n = sqrt( mu / abs(pow(a, 3)))
	# 1. Calcular la Anomalía Media (M)
	var mean_anomaly = _mean_anomaly_at_t0 + n * (t)
	
	match orbit_type:
		ORBITS_TYPES.RADIAL:
			# La dirección es constante (dada por la posición inicial)
			var r_unit: Vector3 = _initial_position.normalized()
			
			# Necesitas una función que resuelva el movimiento radial unidimensional
			var result = _solve_radial_motion(t, _initial_position.length(), _initial_velocity.dot(r_unit), mu)
			
			var vel_magnitude = result.velocity_magnitude
			vel_perifocal = r_unit * vel_magnitude
	
		ORBITS_TYPES.ELIPTIC:
			mean_anomaly = fmod(mean_anomaly, 2.0 * PI)
			if mean_anomaly < 0.0: mean_anomaly += 2.0 * PI
			
			# 2. M -> E (Anomalía Excéntrica)
			var E = _solve_kepler_newton_raphson(mean_anomaly, e)
			
			# 4. Velocidad (v_vec) en el Plano Perifocal (xp_dot, yp_dot)
			var Edot = n / (1.0 - e * cos(E)) # Derivada de E respecto al tiempo
			
			var xp_dot = -a * sin(E) * Edot
			var yp_dot = a * sqrt(1.0 - e*e) * cos(E) * Edot
			vel_perifocal = Vector3(xp_dot, yp_dot, 0.0)
		
		ORBITS_TYPES.HYPERBOLIC:
			# 2. M -> F (Anomalía Excéntrica Hiperbólica)
			var F = _solve_kepler_hyperbolic(mean_anomaly, e) # ¡CORRECCIÓN APLICADA!
			
			# Nota: En hiperbólica, 'a' debe ser usado con valor absoluto
			# Aunque lo calculamos como positivo, aseguramos la definición
			var abs_a = abs(a) 
		
			# Velocidad Hiperbólica (se usa n / (e * cosh(F) - 1.0) para F_dot)
			var F_dot = n / (e * cosh(F) - 1.0)
		
			var xp_dot = -abs_a * sinh(F) * F_dot 
			var yp_dot = abs_a * sqrt(e*e - 1.0) * cosh(F) * F_dot
			vel_perifocal = Vector3(xp_dot, yp_dot, 0.0)
		
		ORBITS_TYPES.PARABOLIC:
			return _temp_velocity
			
	return _perifocal_transform.basis * vel_perifocal

func _calculate_influence_radius(M:float, m:float) -> float:
	if not is_massive:
		return 0.0
	var a = abs(semi_major_axis)
	var mass_ratio:float = m / M
	if mass_ratio > 0.17: mass_ratio = 0.17
	var r:float = a * pow(mass_ratio, (2.0/5.0))
	if _show_log_msgs: print(name,". radius of influence: ",r)
	return r

#--------------------------------------- Calculo Orbital ------------------------------------------#
# --- Elementos Orbitales ---
var mu_attractor: float # μ = G * Mass_attractor
var semi_major_axis: float
var eccentricity: float
var true_anomaly: float # Anomalía verdadera (ν)
var period: float # Período orbital (solo para órbitas cerradas)

var _mean_anomaly_at_t0: float = 0.0 # El offset M0
var _eliptic: bool

# ====================================================================
# --- LÓGICA ORBITAL ---
# ====================================================================

func _calculate_orbital_elements(r_vec: Vector3, v_vec:Vector3, attractor: OrbitalObject3D) -> int:
	if not attractor:
		semi_major_axis = INF
		return ORBITS_TYPES.INDETERMINATE
		
	# Calcular mu_attractor a partir de la masa del atractor y G global
	mu_attractor = OrbitalManager.G * attractor.mass
	var r = r_vec.length()
	var v = v_vec.length()
	var mu = mu_attractor
	
	# 1. Momento Angular Específico (h)
	var h_vec = r_vec.cross(v_vec)
	var h = h_vec.length()
	
	# Caso de orbita Radial, v alineado con r o v es nulo
	const H_MAGNITUDE_TOLERANCE = 0.001
	
	if r * v > 0.0:
		var relative_h = h / (r * v)
		if relative_h < H_MAGNITUDE_TOLERANCE:
			transform = get_parent().transform
			return ORBITS_TYPES.RADIAL
	if v < 1e-6:
		transform = get_parent().transform
		return ORBITS_TYPES.RADIAL
	
	# 2. Vector de Excentricidad (e)
	var e_vec = ((v_vec.cross(h_vec)) / mu) - (r_vec.normalized())
	eccentricity = e_vec.length()
	# Rotar el nodo al plano perifocal
	_perifocal_transform = _calculate_orbital_transform(h_vec, e_vec, attractor)

	# 3. Energía Orbital Específica (epsilon) y Eje Semimayor (a)
	var epsilon = (v*v / 2.0) - (mu / r)
	# Lógica para manejar los tres tipos de órbita basados en epsilon:
	if abs(epsilon) < 1e-3 * mu / r:
		# **Órbita Parabólica (Escape Justo):** a = INF
		if _show_log_msgs: print("Parabolic Orbit")
		var dv:float = 0.001
		var result: int
		if _eliptic:
			result = calcule_orbit(r_vec,v_vec * (1.0 + dv))
		else:
			result = calcule_orbit(r_vec,v_vec * (1.0 - dv))
		orbit_type = result
		return result
	elif epsilon < 0:
		# **Órbita Elíptica (Cerrada):** a > 0
		semi_major_axis = -mu / (2.0 * epsilon)
		period = 2.0 * PI * sqrt(pow(semi_major_axis, 3) / mu)
		_eliptic = true
		if _show_log_msgs: print(name, ": Elliptical orbit. Period: ", period, " s, e: ", eccentricity)
	else: # epsilon > 0
		# **Órbita Hiperbólica (Abierta):** a < 0
		semi_major_axis = -mu / (2.0 * epsilon) # Nota: 'a' es negativo  en esta definición hiperbólica
		period = INF # Abierta
		_eliptic = false
		if _show_log_msgs: print(name, ": Hyperbolic orbit. Escape trajectory. e=", eccentricity)
	
	# 7. Anomalía Verdadera (ν)
	true_anomaly = acos(e_vec.dot(r_vec) / (eccentricity * r))
	
	if r_vec.dot(v_vec) < 0:
		# Si r.v < 0, el cuerpo se acerca al periapsis. nu debe ser negativo para hiperbólicas.
		if eccentricity > 1.0:
			true_anomaly = -true_anomaly # Rango correcto para F (negativo antes del periapsis)
		else:
			true_anomaly = (2.0 * PI) - true_anomaly # Mantiene el rango [0, 2pi] para elípticas
	
	# Calcular M0 (Anomalía Media Inicial)
	_mean_anomaly_at_t0 = _true_to_mean_anomaly(true_anomaly, eccentricity)
	
	if _eliptic:
		return ORBITS_TYPES.ELIPTIC
	return ORBITS_TYPES.HYPERBOLIC

func _calculate_orbital_transform(h_vec: Vector3, e_vec: Vector3, attractor: OrbitalObject3D) -> Transform3D:
	# 1. Ejes del Sistema Perifocal
	var k_axis: Vector3 = h_vec.normalized()  # Z (Normal al plano)
	var i_axis: Vector3 = e_vec.normalized()  # X (Al Periapsis)
	var j_axis: Vector3 = k_axis.cross(i_axis) # Y (Complementario)
	# 2. Construir la Basis (Rotación)
	var per_basis = Basis(i_axis, j_axis, k_axis).orthonormalized()
	return Transform3D(per_basis, attractor.position)

# Convierte la Anomalía Verdadera (nu) a Anomalía Media (M)
func _true_to_mean_anomaly(nu: float, e: float) -> float:
	if e == 1.0: 
		return 0.0 # Parabólica no tiene M definido
	
	if e < 1.0: # Órbita Elíptica (nu -> E -> M)
		# 1. nu -> E (Anomalía Excéntrica) usando ATAN2
		var E_y = sin(nu) * sqrt(1.0 - e*e)
		var E_x = cos(nu) + e
		var E = atan2(E_y, E_x)
		
		# Asegurar que E esté en el rango [0, 2PI]
		E = fmod(E, 2.0 * PI)
		if E < 0.0: E += 2.0 * PI
		
		# 2. E -> M (Anomalía Media)
		var M = E - e * sin(E)
		
		# Asegurar que M esté en el rango [0, 2PI]
		M = fmod(M, 2.0 * PI)
		if M < 0.0: M += 2.0 * PI
		
		return M

	else: # Órbita Hiperbólica (e > 1.0): (nu -> F -> M)
		# 1. nu -> F (Anomalía Excéntrica Hiperbólica)
		# Usando la fórmula F = acosh((e + cos(nu)) / (1 + e * cos(nu)))
		var cos_nu = cos(nu)
		var arg = (e + cos_nu) / (1.0 + e * cos_nu)
		
		# acosh(x) = log(x + sqrt(x*x - 1))
		var F_magnitude = log(arg + sqrt(arg * arg - 1.0))
		# El signo de F debe ser el mismo que el signo de nu (o r_vec.dot(v_vec))
		# Como corregimos nu a negativo si r.v < 0, podemos usar nu.
		var F = F_magnitude
		if nu < 0.0: # Si estamos acercándonos al periapsis (nu negativo)
			F = -F_magnitude
		
		# 2. F -> M (Anomalía Media)
		var M = e * sinh(F) - F
		
		return M

func _solve_radial_motion(t_total: float, r0: float, v0: float, mu: float) -> Dictionary:
	
	var time_remaining = t_total
	var r = r0
	var v = v0
	
	# Usaremos el paso de tiempo fijo del motor de Godot para la integración.
	var dt = 0.1
	
	# 🚨 Es crucial que dt sea pequeño, aquí usamos el paso de tiempo del motor
	
	while time_remaining > 0.0:
		var step = min(time_remaining, dt)
		
		# Calcula la aceleración
		var a = -mu / (r * r)
		
		# Integración de Euler (simple)
		r += v * step
		v += a * step
		
		time_remaining -= step
		
		# Evitar r <= 0 (Impacto o Paso Excesivo)
		if r <= 0.0:
			r = 0.0
			v = 0.0
			break
		
	return {
	"position_magnitude": r, 
	"velocity_magnitude": v
	}

func _solve_kepler_newton_raphson(mean_anomaly: float, e: float) -> float:
	
	const MAX_ITERATIONS = 10
	const TOLERANCE = 1e-12
	const MIN_F_PRIME = 0.001 # Nuevo: Mínimo para el denominador
	const MAX_DELTA_E = 1.0   # Nuevo: Máximo paso permitido
	
	var M = fmod(mean_anomaly, 2.0 * PI)
	if M < 0.0:
		M += 2.0 * PI

	# 2. Aproximación Inicial (E = M es un buen punto de partida seguro)
	var E = M
	
	# 3. Bucle de Iteración de Newton-Raphson
	for i in range(MAX_ITERATIONS):
		
		var sin_E = sin(E)
		var cos_E = cos(E)
		
		# La función de Kepler: f(E) = E - e*sin(E) - M
		var f_E = E - e * sin_E - M
		
		# 4. Comprobar la Convergencia (¡Usar f_E para la salida!)
		if abs(f_E) < TOLERANCE:
			break
			
		# La derivada: f'(E) = 1 - e*cos(E)
		var f_prime_E = 1.0 - e * cos_E
		
		# 5. 🚨 Seguridad y Limitación del Denominador
		# Si f'(E) se acerca mucho a cero (problema de alta excentricidad), lo limitamos
		if abs(f_prime_E) < MIN_F_PRIME:
			f_prime_E = MIN_F_PRIME if f_prime_E >= 0.0 else -MIN_F_PRIME
		
		# Cálculo del cambio (delta E)
		var delta_E = f_E / f_prime_E
		
		# 6. 🚨 Limitar el tamaño del paso (para asegurar convergencia)
		if abs(delta_E) > MAX_DELTA_E:
			delta_E = sign(delta_E) * MAX_DELTA_E
		
		# Aplicar la Corrección
		E -= delta_E
		
	# 7. Normalizar el resultado (Mantener entre 0 y 2PI)
	E = fmod(E, 2.0 * PI)
	if E < 0.0:
		E += 2.0 * PI
		
	return E

# Resuelve la Ecuación de Kepler Hiperbólica (M = e*sinh(F) - F) 
func _solve_kepler_hyperbolic(mean_anomaly: float, e: float) -> float:
	
	# Parámetros de Precisión y Límite
	const MAX_ITERATIONS = 15     # Se permite más iteraciones ya que la convergencia puede ser más lenta.
	const TOLERANCE = 1e-12        # Tolerancia para la convergencia
	
	var M = mean_anomaly 
	
	# 1. Aproximación Inicial (Guess F0)
	# Una aproximación común para M grande o e cerca de 1 es usar la misma M.
	# Usaremos una aproximación robusta simple.
	var F = M 
	
	# Una mejor aproximación inicial para F:
	# F = log(2 * M / e + 1.8) (o log(2*M/e + 1) si M es grande, pero M puede ser pequeño)
	if e > 1.0:
		F = log(abs(M / e) + 1.0) # Mejor aproximación para M>0
	
	# Si M es negativo (viaja en la rama opuesta), F también debería serlo.
	if M < 0.0:
		F *= -1.0
	# 2. Bucle de Iteración de Newton-Raphson
	for i in range(MAX_ITERATIONS):
		var sinh_F = sinh(F)
		var cosh_F = cosh(F)
		# La función a resolver: f(F) = e*sinh(F) - F - M
		var f_F = e * sinh_F - F - M
		
		# La derivada: f'(F) = e*cosh(F) - 1
		var f_prime_F = e * cosh_F - 1.0
		
		# Cálculo del cambio (delta F): dF = f(F) / f'(F)
		var delta_F = f_F / f_prime_F
		
		# 3. Comprobar la Convergencia
		if abs(delta_F) < TOLERANCE:
			break
		
		# 4. Aplicar la Corrección
		F -= delta_F
	return F

func _is_eliptic() -> bool:
	return _eliptic

#--------------------------- Fin Calculo Orbital ----------------------------------#

## Returns the body's current orbital velocity, calculated as a Vector3
func get_velocity() -> Vector3:
	var current_time = OrbitalManager.get_current_time() - _initial_time
	_temp_velocity = _get_velocity_at_time(current_time)
	return _temp_velocity

## Returns the eccentricity e of the current orbit as a float. This value determines the shape of the conic section (e.g., e < 1.0 for elliptic, e = 1.0 for parabolic).
func get_eccentricity() -> float:
	return eccentricity

## Returns the orbital period in seconds. Only valid for closed elliptical orbits.
func get_period() -> float:
	return period

## Returns the semi-major axis a of the current orbit as a float. This value determines the size of the orbit and is critical for calculating the period and energy.
func get_semi_major_axis() -> float:
	return semi_major_axis

## Returns the perifocal vector as a Vector3 normalized. This vector defines the X-axis of the body's local orbital frame (perifocal system), pointing toward the periapsis.
func get_perifocal_vector() -> Vector3:
	return _perifocal_transform.basis * Vector3.RIGHT

## Returns the orbital normal vector as a Vector3. This vector defines the Z-axis of the body's local orbital frame (perifocal system).
func get_normal_vector() -> Vector3:
	return _perifocal_transform.basis * Vector3.BACK

## Returns the currently active orbital classification of the body as an ORBITS_TYPES enum value.
func get_orbit_type() -> ORBITS_TYPES:
	return orbit_type

## Returns the current gravitational center of reference (the primary body) as an OrbitalBody3D instance. This is the node the body is currently orbiting.
func get_attractor() -> OrbitalObject3D:
	return attractor

## Registers a new OrbitalBody3D as a child orbiter of this node and sets its initial conditions for the two-body problem. This function performs the necessary transformation to switch the orbiter's frame of reference, setting its attractor property to the current node, and defining its state via new_position and new_velocity vectors relative to the new attractor.
func add_orbiter(orbiter:Node, new_position:Vector3, new_velocity:Vector3) -> void:
	if not orbiter is OrbitalObject3D: return
	var old_attractor: OrbitalObject3D = orbiter.attractor
#	if old_attractor: old_attractor.remove_orbiter(orbiter)
	orbiter.attractor = self
	orbiter._initial_position = new_position
	orbiter._initial_velocity = new_velocity
	orbiter.orbit_type = ORBITS_TYPES.INDETERMINATE
	if orbiter.get_parent() != self:
		if orbiter.get_parent():
			orbiter.reparent(self, true)
	_orbiters.append(orbiter)
	if orbiter.is_massive:
		_massive_orbiters.append(orbiter)
	if _show_log_msgs: print(orbiter.name, " added as an orbiter of the ", self.name)
	orbiter_added.emit(orbiter)
	orbiter.has_new_attractor.emit(self)

## Removes the specified OrbitalBody3D from this node's list of controlled orbiters. This is typically called when an orbiter is exiting the current gravitational sphere of influence (SOI) and transferring to a new attractor. Note that this function only updates the internal list and does not automatically change the orbiter's attractor property.
func remove_orbiter(orbiter:OrbitalObject3D) -> void:
	_orbiters.erase(orbiter)
	if orbiter.is_massive:
		_massive_orbiters.erase(orbiter)
	orbiter_removed.emit(orbiter)

## Returns true if this body currently has at least one satellite/orbiter inside its Sphere of Influence.
func has_orbiters() -> bool:
	return _orbiters.size() > 0

func has_massive_orbiters() -> bool:
	return _massive_orbiters.size() > 0

## Returns an array containing all bodies currently orbiting this object (inside its Sphere of Influence).
func get_orbiters() -> Array:
	return orbiters

## Initializes and calculates all Classical Orbital Elements (COEs) for the body's new state relative to its current attractor. This function is used when the body enters a new sphere of influence or is instantiated, and it establishes the necessary parameters (e.g., eccentricity, semi-major axis, energy) required for the subsequent propagation of the orbit. It automatically starts the orbital simulation time counter and updates the body's influence radius. Returns the ORBITS_TYPES classification of the trajectory established by the given state vectors.
func calcule_orbit(new_position: Vector3, new_velocity: Vector3) -> ORBITS_TYPES:
	if not attractor:
		if _show_log_msgs: print(name, ": Body without attractor, central body.")
		return ORBITS_TYPES.CENTRAL_OBJECT
	
	var o_t:ORBITS_TYPES = _calculate_orbital_elements(new_position, new_velocity, attractor)
	
	if o_t:
		_initial_time = OrbitalManager.get_current_time()
		influence_radius = _calculate_influence_radius(attractor.mass, mass)
		orbit_changed.emit()
	return o_t

## Calculates the position of the orbital path and returns it as a sequence of coordinates in the current attractor's frame of reference. This method propagates the body's current orbit into a list of points suitable for 3D rendering. The number of points returned is defined by the segments parameter, which controls the graphical resolution of the trajectory path.
func get_trajectory(segments: int) -> Array[Vector3]:
	var result:Array[Vector3] = []
	var a = abs(semi_major_axis)
	var e = eccentricity
	var point:Vector3
	var SOI_radius:float = attractor.influence_radius
	var central:bool = attractor.orbit_type == ORBITS_TYPES.CENTRAL_OBJECT
	match orbit_type:
		ORBITS_TYPES.ELIPTIC:
			var E:float = -PI
			var dE: float = 2 * PI / float(segments)
			while E < PI:
				var xp = a * (cos(E) - e)
				var yp = a * sqrt(1.0 - e*e) * sin(E)
				point = _perifocal_transform.basis * Vector3(xp, yp, 0.0)
				if central or point.length() <= SOI_radius:
					result.append(point)
				E += dE
			point = _perifocal_transform.basis * Vector3(a*(-1 - e), 0.0, 0.0)
			if central or point.length() <= SOI_radius:
				result.append(point)
			return result
		ORBITS_TYPES.HYPERBOLIC:
			# Ángulo máximo de la asíntota (en radianes)
			var nu_max = acos(-1.0 / e) * 0.99
			# Queremos puntos simétricos: desde -nu_max hasta +nu_max
			for i in range(segments + 1):  # +1 para cerrar el último punto si quieres
				var t = float(i) / float(segments)           # 0.0 → 1.0
				var nu = -nu_max + 2.0 * nu_max * t          # ν de -nu_max a +nu_max
				
				var r = a * (e * e - 1.0) / (1.0 + e * cos(nu))
				
				# Posición en el plano orbital (pericentro en X)
				var xp = r * cos(nu)
				var yp = r * sin(nu)
				point = _perifocal_transform.basis * Vector3(xp, yp, 0.0)
				if central or point.length() <= SOI_radius:
					result.append(point)
			return result
	return result

## Returns the relative inclination between this body's orbital plane and the plane of the other_orbiter in radians. This value is the angle between their respective orbital normal vectors. Returns 0.0 if the two orbiters do not share the same gravitational attractor.
func inclination_to(other_orbiter:OrbitalObject3D) -> float:
	if not other_orbiter.attractor == attractor:
		return 0.0
	var other_normal:Vector3 = other_orbiter.get_normal_vector()
	return other_normal.angle_to(get_normal_vector())

## Calculates and returns the nodal axis (or Line of Nodes) between this orbit and the orbit of the other_orbiter as a Vector3. The line of nodes is the vector defined by the intersection of the two orbital planes. Returns Vector3.ZERO if the two orbiters do not share the same gravitational attractor.
func nodal_axis_to(other_orbiter:OrbitalObject3D) -> Vector3:
	if not other_orbiter.attractor == attractor:
		return Vector3.ZERO
	var other_normal:Vector3 = other_orbiter.get_normal_vector()
	return other_normal.cross(get_normal_vector())

## Returns the position of the periapsis (closest point to the attractor) for the current orbit. Returns Vector3.ZERO for circular, parabolic, or invalid orbits.
func get_periapsis() -> Vector3:
	var result:Vector3 = Vector3.ZERO
	var a = semi_major_axis
	var e = eccentricity
	match orbit_type:
		ORBITS_TYPES.ELIPTIC:
			var xp = a * (1 - e)
			if name == "Ship": print("periapsis: ", xp)
			result = _perifocal_transform.basis * Vector3(xp, 0.0, 0.0)
			return result
		ORBITS_TYPES.HYPERBOLIC:
			var xp = a * (e * e - 1.0) / (1.0 + e)
			result = _perifocal_transform.basis * Vector3(xp, 0.0, 0.0)
			return result
	return result

## Returns the position of the apoapsis (farthest point from the attractor) for the current orbit. Only valid for elliptical orbits. Returns Vector3.ZERO for all other orbit types.
func get_apoapsis() -> Vector3:
	var result:Vector3 = Vector3.ZERO
	match orbit_type:
		ORBITS_TYPES.ELIPTIC:
			var a = semi_major_axis
			var e = eccentricity
			var xp = a * (-1 - e)
			result = _perifocal_transform.basis * Vector3(xp, 0.0, 0.0)
			return result
	return result

## It returns gravitational force for use in a non-inertial state.
func get_force() -> Vector3:
	var f = mu_attractor * mass / position.length() / position.length()
	return - position.normalized() * f

func _soi_entered(node: Node3D) -> void:
	if not node is OrbitalObject3D:
		return
	var new_orbiter: OrbitalObject3D = node
	if not new_orbiter.orbit_type:
		return
	if new_orbiter.mass >= mass:
		return
	if new_orbiter.attractor == attractor:
		var new_pos:Vector3 = new_orbiter.position - position
		var new_vel:Vector3 = new_orbiter.get_velocity() - get_velocity()
		
		add_orbiter(new_orbiter, new_pos, new_vel)
		
		if _show_log_msgs: print(new_orbiter.name," entering the orbit of the ",name)
	return

func _soi_exited(node: Node3D) -> void:
	if not node is OrbitalObject3D:
		return
	if not is_instance_valid(attractor):
		return
	var orbiter:OrbitalObject3D = node
	if not orbiter.orbit_type:
		return
	if not orbiter. attractor == self:
		return
	if orbiter.position.length() < influence_radius:
		return
	var new_attractor: OrbitalObject3D = attractor
	var new_pos:Vector3 = orbiter.position + position
	var new_vel:Vector3 = orbiter.get_velocity() + get_velocity()
	
	new_attractor.add_orbiter(orbiter,new_pos, new_vel)
	
	if _show_log_msgs: print(node.name," leaving the orbit of the ", name)

## Function that checks if there are bodies entering or leaving the SOI radius of influence.
func _chek_orbiters():
	if is_massive and orbit_type != ORBITS_TYPES.CENTRAL_OBJECT:
		for orb:OrbitalObject3D in orbiters:
			if orb.position.length() > influence_radius:
				_soi_exited(orb)
	if not has_massive_orbiters(): return
	for att:OrbitalObject3D in massive_orbiters:
		for orb:OrbitalObject3D in orbiters:
			if orb == att: continue
			if (orb.position - att.position).length() < att.influence_radius:
				att._soi_entered(orb)
