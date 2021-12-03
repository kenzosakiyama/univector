from navigation.arena_utils import *
from navigation.un_field import *
from linear_algebra_2D.linalg import Vec2D

# TODO: Talvez no futuro seja bom por que Navigator seja uma classe abstrata
class Navigator:

    '''
        Classe responsável por interfacear o Univector.
    '''

    def __init__(self, team_side: TeamSide,
                       univector_parameters_path: str,):

        self.team_side = team_side
        self.attack_goal = TeamSide.RIGHT if team_side == TeamSide.LEFT else TeamSide.LEFT
        self.univector = UnivectorField.from_json(univector_parameters_path)
    

    def _get_correct_axis(self, position: Vec2D, section_num: ArenaSections,
                         attack_goal: bool = RIGHT) -> Vec2D:

        axis = Axis[section_num.value]

        if attack_goal == LEFT:
            axis = axis * -1

        return axis

    def _get_correct_offset(self, position: Vec2D, section_num: ArenaSections) -> Vec2D:
        return Offsets[section_num.value]

    # TODO: esse é um bom nome para o método?
    def get_angle(self, origin_position: Vec2D,
                        target_position: Vec2D,
                        obstacles_pos: List[Vec2D],
                        obstacles_speed: List[Vec2D] = None) -> float:

        # Método reponsável por determinar o ângulo de aproxição do robô ao utilizar o Univector

        section_num = univector_pos_section(target_position)

        if section_num == ArenaSections.CENTER:
            goal_pos = goal_position(self.team_side)
            correct_axis = goal_pos - target_position
        else:
            if self.attack_goal == TeamSide.RIGHT:
                # print("uai")
                if section_num == ArenaSections.RIGHT_DOWN_CORNER or section_num == ArenaSections.RIGHT_UP_CORNER:
                    print("canto esquerdo")
                    correct_axis = Vec2D.right()
                else:
                    correct_axis = self._get_correct_axis(target_position, section_num, self.attack_goal)                    
            else:
                if section_num == ArenaSections.LEFT_DOWN_CORNER or section_num == ArenaSections.LEFT_UP_CORNER:
                    correct_axis = Vec2D.left()
                else:
                    correct_axis = self._get_correct_axis(target_position, section_num, self.attack_goal)     
        
        offset = self._get_correct_offset(target_position, section_num) 
        approach_angle = (correct_axis + offset).angle()

        return self.univector.get_angle(
            origin_position,
            target_position,
            approach_angle,
            obstacles_pos,
            obstacles_speed
        )
        