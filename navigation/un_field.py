from typing import List, Tuple
import math
from math import pi
from math import cos, sin, atan2, exp, sqrt
# TODO: Atualizar o caminho ao integrar ao sistema da Ararabots
from linear_algebra_2D.linalg import *

LEFT = 0
RIGHT = 1

def gaussian(m, v):
    return exp(-(m**2) / (2 * (v**2)))

def wrap2pi(theta: float) -> float:
    if theta > pi:
        return theta - 2 * pi
    if theta < -pi:
        return 2 * pi + theta
    else:
        return theta

    
def create_unit_vector_from_angle(theta: float) -> Vec2D:
    # radianos
    # retorna vetor unitário com a orientação especificada
    return Vec2D(cos(theta), sin(theta))

class HyperbolicSpiral:

    def __init__(self, _Kr: float, _radius: float):
        self.Kr: float = _Kr
        self.radius: float = _radius

    def update_params(self, _KR: float, _RADIUS: float) -> None:
        self.Kr = _KR
        self.radius = _RADIUS

    def fi_h(self, _p: Vec2D, radius: float = None, cw: bool = True) -> float:

        if radius is None:
            r = self.radius
        else:
            r = radius

        p = _p
        theta = atan2(p[1], p[0])
        ro = p.norm()

        if ro > r:
            a = (pi / 2.0) * (2.0 - (r + self.Kr) / (ro + self.Kr))
        else:
            a = (pi / 2.0) * math.sqrt(ro / r)

        if cw:
            _theta = wrap2pi(theta + a)
        else:
            _theta = wrap2pi(theta - a)

        return atan2(sin(_theta), cos(_theta))

    def n_h(self, _p: Vec2D, _radius: float = None, cw: bool = True) -> Vec2D:
        p = _p
        if _radius is None:
            radius = self.radius
        else:
            radius = _radius

        fi = self.fi_h(p, radius, cw)
        return Vec2D(cos(fi), sin(fi))


class Repulsive:

    def __init__(self):
        self.origin = Vec2D.origin()

    def update_origin(self, newOrigin: Vec2D) -> None:
        self.origin = newOrigin.copy()

    def fi_r(self, _p, _origin: Vec2D = None, _theta: bool = True) -> Vec2D:
        if _origin is not None:
            self.update_origin(_origin)

        p = _p - self.origin

        if _theta:
            return atan2(p[1], p[0])
        else:
            return p


class Move2Goal:

    def __init__(self, _Kr: float, _radius: float):

        self.Kr: float = _Kr
        self.radius: float = _radius
        
        self.hyperSpiral = HyperbolicSpiral(self.Kr, self.radius)
        self.origin = Vec2D.origin()

        self.u = Vec2D.origin()
        self.v = Vec2D.origin()

        self.toUnivectorMatrix = None
        self.toCanonicalMatrix = None

    def update_params(self, _KR: float, _RADIUS: float) -> None:
        self.Kr = _KR
        self.radius = _RADIUS
        self.hyperSpiral.update_params(self.Kr, self.radius)

    def update_axis(self, new_origin: Vec2D, new_u_axis: Vec2D) -> None:
        self.origin = new_origin
        self.u = new_u_axis
        self.build_axis()

    def build_axis(self) -> None:
        self.u /= -self.u.norm()
        theta = math.atan2(self.u[1], self.u[0])
        self.v = Vec2D(-sin(theta), cos(theta))

        self.toCanonicalMatrix = Mat2D(self.u, self.v)
        self.toUnivectorMatrix = self.toCanonicalMatrix.invert()

    def fi_tuf(self, _p: Vec2D) -> float:
        n_h = self.hyperSpiral.n_h
        p = _p - self.origin
        r = self.radius

        p = self.toUnivectorMatrix.transform(p)

        x, y = p
        yl = y + r
        yr = y - r

        # Parece que houve algum erro de digitacao no artigo
        # Pois quando pl e pr sao definidos dessa maneira o campo gerado
        # se parece mais com o resultado obtido no artigo
        pl = Vec2D(x, yr)
        pr = Vec2D(x, yl)

        # Este caso eh para quando o robo esta dentro do "circulo" da bola
        if -r <= y < r:
            nh_pl = n_h(pl, cw=False)
            nh_pr = n_h(pr, cw=True)

            # Apesar de no artigo nao ser utilizado o modulo, quando utilizado
            # na implementacao o resultado foi mais condizente com o artigo
            vec = (abs(yl) * nh_pl + abs(yr) * nh_pr) / (2.0 * r)
            vec = self.toCanonicalMatrix.transform(vec)
        else:
            if y < -r:
                theta = self.hyperSpiral.fi_h(pl, cw=True)
            else:  # y >= r
                theta = self.hyperSpiral.fi_h(pr, cw=False)

            vec = Vec2D(cos(theta), sin(theta))
            # TODO: MATRIZES
            vec = self.toCanonicalMatrix.transform(vec)

        return atan2(vec[1], vec[0])


class AvoidObstacle:
    def __init__(self, _pObs: Vec2D, _vObs: Vec2D, _pRobot: Vec2D, _vRobot: Vec2D, _K0: float):
        self.pObs =_pObs.copy()
        self.vObs = _vObs.copy()
        self.pRobot = _pRobot.copy()
        self.vRobot = _vRobot.copy()
        self.K0 = _K0
        self.repField = Repulsive()

    def get_s(self) -> Vec2D:
        return self.K0 * (self.vObs - self.vRobot)

    def get_virtual_pos(self) -> Vec2D:
        s = self.get_s()
        s_norm = s.norm()
        d = (self.pObs - self.pRobot).norm()
        if d >= s_norm:
            v_pos = self.pObs + s
        else:
            v_pos = self.pObs + (d / s_norm) * s
        return v_pos

    def fi_auf(self, _robotPos: Vec2D, _vPos: Vec2D = None, _theta: bool = True) -> Vec2D:
        if _vPos is None:
            v_pos = self.get_virtual_pos()
        else:
            v_pos = _vPos
        vec = self.repField.fi_r(_robotPos, _origin=v_pos, _theta=_theta)
        return vec

    def update_param(self, _K0: float) -> None:
        self.K0 = _K0

    def update_obstacle(self, _pObs: Vec2D, _vObs: Vec2D) -> None:

        self.pObs = _pObs
        self.vObs = _vObs

    def update_robot(self, _pRobot: Vec2D, _vRobot: Vec2D) -> None:

        self.pRobot = _pRobot
        self.vRobot = _vRobot


class UnivectorField:
    def __init__(self, RADIUS: float,
                       KR: float,
                       K0: float,
                       DMIN: float,

                       LDELTA: float):

        # Constantes
        self.RADIUS: float = RADIUS
        self.KR: float = KR
        self.K0: float = K0
        self.DMIN: float = DMIN
        self.LDELTA: float = LDELTA

        # Subfields
        self.avdObsField = AvoidObstacle(Vec2D.origin(), Vec2D.origin(),
                                         Vec2D.origin(), Vec2D.origin(), self.K0)

        self.mv2Goal = Move2Goal(self.KR, self.RADIUS)

    # Definição de métodos "construtores" a partir de arquivo e JSON
    @classmethod
    def from_json(cls, parameters_path: str):
        # Cria uma instância do Univector a partir de um arquivo de parametros
        import json

        with open(parameters_path, 'r') as f:
            parameters = json.load(f)

        return cls(**parameters)
    
    @classmethod
    def from_dict(cls, parameters_dict: str):
        # Cria uma instância do Univector a partir de um dicionário com os parâmetros. 
        # Utilize os mesmos nomes do contrutor do Univcetor para as chaves!
        return cls(**parameters_dict)

    @staticmethod
    def get_attack_goal_axis(attack_goal: bool) -> Vec2D:
        if attack_goal == LEFT:
            return Vec2D.left()
        else:
            return Vec2D.right()

    @staticmethod
    def get_attack_goal_position(attack_goal: bool) -> Vec2D:
        """
        Return the position of the goal, given attacking side  and section of the object
        :param team_side: int
        :return: np.array([x,y])
        """
        return Vec2D(attack_goal * 150, 65)

    def update_constants(self, _RADIUS: float, _KR: float, _K0: float, _DMIN: float, _LDELTA: float) -> Vec2D:
        self.RADIUS = _RADIUS
        self.KR = _KR
        self.K0 = _K0
        self.DMIN = _DMIN
        self.LDELTA = _LDELTA

        self.avdObsField.update_param(self.K0)
        self.mv2Goal.update_params(self.KR, self.RADIUS)

    def _get_repulsive_centers(self, obstacles_pos: List[Vec2D],
                                     obstacles_speed: List[Vec2D]) -> List[Vec2D]:

        centers = []

        for obs_pos, obs_speed in zip(obstacles_pos, obstacles_speed):

            self.avdObsField.update_obstacle(obs_pos, obs_speed)
            center = self.avdObsField.get_virtual_pos()
            centers.append(center)
        
        return centers

    def _get_nearest_center(self, origin_position: Vec2D, centers: List[Vec2D]) -> Tuple[float, Vec2D]:

        nearest_dist = math.inf
        nearest_center = None

        for center in centers:
            current_dist = (origin_position - center).norm()

            if current_dist < nearest_dist:
                nearest_dist = current_dist
                nearest_center = center 
        
        return nearest_dist, nearest_center

    def get_angle(self, origin_pos: Vec2D, 
                        target_pos: Vec2D,
                        desired_approach_angle: float,
                        obstacles_pos: List[Vec2D],
                        obstacles_speed: List[Vec2D] = None,
                        add_border_obst: bool = True) -> float:
        # Método principal que irá executar o algoritmo de navegação
        vec2follow = create_unit_vector_from_angle(desired_approach_angle)
        self.mv2Goal.update_axis(target_pos, vec2follow)


        # Verifica se as velocidades foram fornecidas. Caso contrário, utiliza-se velocidades nulas.
        if obstacles_speed is None:
            obstacles_speed = [Vec2D.origin() for _ in range(len(obstacles_pos))]

        # Inicialização de variáveis
        closest_center = None  # array to store the closest center
        centers = []
        min_distance = self.DMIN + 1

        # TODO: adicionando bordas como sendo "obstáculos"
        # TODO: se funcionar da para remover o len(obstacles_pos)
        # Adicionando bordas da arena como "obstáculos", utilizando a posição X da origem da navegação como referência
        if add_border_obst:
            obstacles_pos = obstacles_pos.copy()
            obstacles_pos.append( Vec2D(origin_pos[0], 0) )   # Borda inferior
            obstacles_pos.append( Vec2D(origin_pos[0], 130) ) # Borda superior

        has_obstacles = len(obstacles_pos) > 0

        # Caso existam obstáculos
        if has_obstacles:
            centers = self._get_repulsive_centers(obstacles_pos, obstacles_speed)

            min_distance, closest_center = self._get_nearest_center(origin_pos, centers)

            fi_auf = self.avdObsField.fi_auf(origin_pos, _vPos=closest_center, _theta=True)

        # Caso o robô esteja próximo a um obstáculo
        if min_distance <= self.DMIN:
            return fi_auf
        # Caso contrário
        else:
            fi_tuf = self.mv2Goal.fi_tuf(origin_pos)
            # Checks if at least one obstacle exist
            if has_obstacles:
                g = gaussian(min_distance - self.DMIN, self.LDELTA)
                diff = wrap2pi(fi_auf - fi_tuf)
                return wrap2pi(g * diff + fi_tuf)
            else:  # if there is no obstacles
                return fi_tuf
