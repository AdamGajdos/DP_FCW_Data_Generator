import json
import random
import xlsxwriter
import math
import numpy


class RoadPointTomTom:
    longitude: float
    latitude: float
    speed_limit: float

    def __init__(self, longitude, latitude, speed_limit):
        self.longitude = longitude
        self.latitude = latitude
        self.speed_limit = speed_limit


class RoadPoint:
    deceleration: float
    relative_velocity: float
    distance: float
    velocity: float
    steep: int
    angle: float
    deceleration_lead: float
    gear: int

    def __init__(self, deceleration, velocity, relative_velocity, distance, steep, angle, deceleration_lead, gear):
        self.deceleration = deceleration
        self.velocity = velocity
        self.relative_velocity = relative_velocity
        self.distance = distance
        self.steep = steep
        self.angle = angle
        self.deceleration_lead = deceleration_lead
        self.gear = gear


class RoadPointFinal:
    longitude: float
    latitude: float
    deceleration: float
    relative_velocity: float
    distance: float
    velocity: float
    steep: int
    angle: float
    deceleration_lead: float
    gear: int
    speed_limit: float

    def __init__(self, longitude, latitude, deceleration, velocity, relative_velocity, distance, steep, angle,
                 deceleration_lead, gear, speed_limit):
        self.longitude = longitude
        self.latitude = latitude
        self.deceleration = deceleration
        self.velocity = velocity
        self.relative_velocity = relative_velocity
        self.distance = distance
        self.steep = steep
        self.angle = angle
        self.deceleration_lead = deceleration_lead
        self.gear = gear
        self.speed_limit = speed_limit


class Point:
    longitude: float
    latitude: float

    def __init__(self, longitude, latitude):
        self.longitude = longitude
        self.latitude = latitude


__SPEED_130 = 36.1111  # in m/s
__SPEED_110 = 30.5556  # in m/s
__SPEED_90 = 25  # in m/s
__SPEED_70 = 19.4444  # in m/s
__SPEED_50 = 13.8889  # in m/s
__SPEED_30 = 8.3333  # in m/s

__SECTOR_LENGTH = 10


class SectorState:
    ACCELERATING = 1
    DECELERATING = 2
    CONSTANT = 3


def set_road_points_tomtom(tomtom_json):
    __tomtom_points = []

    for idx, __point in enumerate(tomtom_json['routes'][0]['legs'][0]['points']):
        tomtom_point = RoadPointTomTom(longitude=__point['longitude'], latitude=__point['latitude'], speed_limit=-1)

        __tomtom_points.append(tomtom_point)

    return __tomtom_points


# calculation from: https://www.movable-type.co.uk/scripts/latlong.html
def calculate_distance_2_points(point_a: Point, point_b: Point) -> float:
    r = 6371000  # in m

    fi_1 = point_a.latitude * math.pi / 180
    fi_2 = point_b.latitude * math.pi / 180
    delta_fi = (point_b.latitude - point_a.latitude) * math.pi / 180

    delta_lambda = (point_b.longitude - point_a.longitude) * math.pi / 180

    a = math.sin(delta_fi / 2) * math.sin(delta_fi / 2) + math.cos(fi_1) * math.cos(fi_2) * \
        math.sin(delta_lambda / 2) * math.sin(delta_lambda / 2)

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    d = r * c  # in m

    return d


def create_scenario(num_of_sectors: int) -> []:
    scenario = []

    scenarios = [SectorState.ACCELERATING, SectorState.DECELERATING, SectorState.CONSTANT]

    for i in range(num_of_sectors):
        scenario.append(scenarios[int(random.uniform(0, scenarios.__len__()))])

    return scenario


def gen_data_for_sector(scenario_state: int, is_first: bool, other_values: dict,
                        max_speed: float, min_speed: float) -> dict:
    result = {
        "start_velocity": 0,
        "start_deceleration": 0,
        "end_velocity": 0,
        "end_deceleration": 0,
        "adjustment_velocity": 0,
        "adjustment_deceleration": 0,
        "scenario_state": scenario_state
    }

    if scenario_state == SectorState.ACCELERATING:
        if is_first:
            start_velocity = random.uniform(min_speed, max_speed)
            start_deceleration = 0
        else:
            start_velocity = other_values.get('velocity')
            start_deceleration = other_values.get('deceleration')

        end_velocity = random.uniform(start_velocity + 5, max_speed)  # 5 m/s -> 18 km/h
        velocity_diff = abs(start_velocity - end_velocity)
        velocity_adjustment = velocity_diff / other_values.get('sector_length')

        points = other_values.get('points')
        t = 0  # time spent traveling between 2 points
        velocity_actual = start_velocity
        for idx, point in enumerate(points):
            if idx > 0:

                s = calculate_distance_2_points(point_a=Point(longitude=points[idx-1].longitude,
                                                              latitude=points[idx-1].latitude),
                                                point_b=Point(longitude=point.longitude,
                                                              latitude=point.latitude))
                t += s / velocity_actual
                velocity_actual += velocity_adjustment

        end_deceleration = velocity_diff / t
        deceleration_diff = abs(start_deceleration - end_deceleration)

        deceleration_adjustment = deceleration_diff / other_values.get('sector_length')

    elif scenario_state == SectorState.DECELERATING:
        if is_first:
            start_velocity = random.uniform(min_speed, max_speed)
            start_deceleration = 0
        else:
            start_velocity = other_values.get('velocity')
            start_deceleration = other_values.get('deceleration')

        end_velocity = random.uniform(min_speed, start_velocity - 5)  # -5 m/s -> 18 km/h
        velocity_diff = abs(start_velocity - end_velocity)
        velocity_adjustment = velocity_diff / other_values.get('sector_length')

        points = other_values.get('points')
        t = 0  # time spent traveling between 2 points
        velocity_actual = start_velocity
        for idx, point in enumerate(points):
            if idx > 0:
                s = calculate_distance_2_points(point_a=Point(longitude=points[idx-1].longitude,
                                                              latitude=points[idx-1].latitude),
                                                point_b=Point(longitude=point.longitude,
                                                              latitude=point.latitude))
                t += s / velocity_actual
                velocity_actual -= velocity_adjustment

        end_deceleration = velocity_diff / t
        deceleration_diff = abs(start_deceleration - end_deceleration)

        deceleration_adjustment = deceleration_diff / other_values.get('sector_length')

    else:
        if is_first:
            start_velocity = random.uniform(min_speed, max_speed)
        else:
            start_velocity = other_values.get('velocity')

        end_velocity = start_velocity
        velocity_diff = abs(start_velocity - end_velocity)
        velocity_adjustment = velocity_diff / other_values.get('sector_length')

        start_deceleration = 0
        end_deceleration = start_deceleration
        deceleration_diff = abs(start_deceleration - end_deceleration)
        deceleration_adjustment = deceleration_diff / other_values.get('sector_length')

    result['start_velocity'] = start_velocity
    result['start_deceleration'] = start_deceleration

    result['end_velocity'] = end_velocity
    result['end_deceleration'] = end_deceleration

    result['adjustment_velocity'] = velocity_adjustment
    result['adjustment_deceleration'] = deceleration_adjustment

    return result


def process_sector(sector_data: dict, sector_length: int) -> []:
    result = []

    velocity = sector_data["start_velocity"]
    velocity_adjustment = sector_data["adjustment_velocity"]
    deceleration = sector_data["start_deceleration"]
    deceleration_adjustment = sector_data["adjustment_deceleration"]
    scenario_state = sector_data["scenario_state"]

    for idx in range(sector_length):

        if velocity <= 4.1667:
            gear = 1
        elif velocity <= 8.3333:
            gear = 2
        elif velocity <= 13.8889:
            gear = 3
        elif velocity <= 19.4444:
            gear = 4
        elif velocity <= 25:
            gear = 5
        else:
            gear = 6

        result.append(RoadPoint(
            deceleration=deceleration,
            deceleration_lead=0,
            velocity=velocity,
            relative_velocity=0,
            distance=150,
            angle=0,
            steep=1,
            gear=gear
        ))

        if scenario_state == SectorState.ACCELERATING:
            velocity += velocity_adjustment
            deceleration -= deceleration_adjustment

        elif scenario_state == SectorState.DECELERATING:
            velocity -= velocity_adjustment
            deceleration += deceleration_adjustment

    return result


def gen_data(scenario: [], max_speed: float, min_speed: float, road_data: []) -> []:
    res_data = [RoadPoint]
    other_values = {}
    num_of_points = road_data.__len__()

    for idx, sector in enumerate(scenario):

        sector_length = __SECTOR_LENGTH

        if idx == (scenario.__len__() - 1):
            if (num_of_points % __SECTOR_LENGTH) < 5:
                sector_length += (num_of_points % __SECTOR_LENGTH)
            elif (num_of_points % __SECTOR_LENGTH) < __SECTOR_LENGTH:
                sector_length = num_of_points % __SECTOR_LENGTH

        start_idx = idx * __SECTOR_LENGTH
        end_idx = start_idx + sector_length - 1

        other_values["sector_length"] = sector_length
        other_values["points"] = road_data[start_idx:end_idx+1]       # new

        if idx == 0:
            is_first = True
        else:
            is_first = False
            other_values["velocity"] = res_data[start_idx - 1].velocity
            other_values["deceleration"] = res_data[start_idx - 1].deceleration

        sector_processed = process_sector(sector_data=gen_data_for_sector(scenario_state=sector,
                                                                          is_first=is_first,
                                                                          min_speed=min_speed,
                                                                          max_speed=max_speed,
                                                                          other_values=other_values),
                                          sector_length=sector_length)

        for point in sector_processed:
            res_data.append(point)

    return res_data


def process_scenario(lead_vehicle: bool, road_data: [], scenario: []) -> []:
    if lead_vehicle:
        min_speed = __SPEED_70
        max_speed = __SPEED_110
    else:
        min_speed = __SPEED_50
        max_speed = __SPEED_90

    complete_scenario = gen_data(scenario=scenario, road_data=road_data,
                                 max_speed=max_speed, min_speed=min_speed)

    complete_scenario.remove(complete_scenario[0])  # first element was empty

    return complete_scenario


def generate_test_data(road_points: []) -> []:
    result = []

    num_of_sectors = int(road_points.__len__() / __SECTOR_LENGTH)

    scenario = create_scenario(num_of_sectors=num_of_sectors)

    our_vehicle = process_scenario(lead_vehicle=False, road_data=road_points, scenario=scenario)

    # scenario = create_scenario(num_of_sectors=num_of_sectors)

    lead_vehicle = process_scenario(lead_vehicle=True, road_data=road_points, scenario=scenario)

    for idx, point in enumerate(our_vehicle):
        point.relative_velocity = lead_vehicle[idx].velocity - point.velocity
        point.deceleration_lead = lead_vehicle[idx].deceleration

        if idx == 0:
            point.distance = 150
        else:
            if -0.5 < point.relative_velocity < 0.5:  # constant motion, distance is preserved
                distance_adjustment = 0
            else:
                # rel_velocity <= -0.5 ... leading vehicle is getting closer
                # rel_velocity >=  0.5 ... leading vehicle is getting away
                s = calculate_distance_2_points(point_a=Point(longitude=road_points[idx - 1].longitude,
                                                              latitude=road_points[idx - 1].latitude),
                                                point_b=Point(longitude=road_points[idx].longitude,
                                                              latitude=road_points[idx].latitude))

                t_our = s / point.velocity  # time needed to travel distance between 2 points in our vehicle
                t_lead = s / lead_vehicle[idx].velocity  # time needed to travel -||- in leading vehicle

                t_diff = t_lead - t_our
                # when lead is faster -> t_diff < 0 (t_lead < t_our) -> v_rel >= 0.5 - AWAY
                # when our is faster -> t_diff > 0 (t_lead > t_our) -> v_rel <= -0.5 - CLOSER

                distance_adjustment = point.relative_velocity * t_diff  # relative distance traveled between 2 points

                if point.relative_velocity >= 0:
                    distance_adjustment *= -1  # the distance is increasing

            point.distance = numpy.clip(a=result[idx - 1].distance + distance_adjustment, a_min=0, a_max=150)
            # point.distance = numpy.clip(a=distance_adjustment, a_min=-150, a_max=150)

        result.append(RoadPointFinal(
            longitude=road_points[idx].longitude,
            latitude=road_points[idx].latitude,
            velocity=point.velocity,
            relative_velocity=point.relative_velocity,
            deceleration=point.deceleration,
            deceleration_lead=point.deceleration_lead,
            distance=point.distance,
            gear=point.gear,
            steep=point.steep,
            angle=point.angle,
            speed_limit=-1
        ))

    return result


def create_testing_data(for_python: bool, source_file_name):
    f_coordinates = open(source_file_name, mode='r')

    coordinates = json.load(f_coordinates)

    tomtom_points = set_road_points_tomtom(coordinates)

    road_points = generate_test_data(road_points=tomtom_points)

    if for_python:

        f_final = open('Road_Data.json', mode='w')

        r_points = []

        for point in road_points:
            r_point = {
                "position": {
                    "longitude": point.longitude,
                    "latitude": point.latitude
                },
                "velocity": point.velocity,
                "acceleration": -point.deceleration,
                "deceleration": point.deceleration,
                "delay": 0.273,  # in ms - average reaction time due to: https://humanbenchmark.com/tests/reactiontime
                "distance": point.distance,
                "relative_velocity": point.relative_velocity,
                "road_info": {
                    "road_type": "ASPHALT",
                    "condition": "DRY"
                },
                "steep": "UPHILL",
                "angle": 0,
                "deceleration_lead": point.deceleration_lead,
                "gear": point.gear
            }

            r_points.append(r_point)

        final_points = {
            'points': r_points
        }

        json_serialized = json.dumps(final_points)

        f_final.write(json_serialized)

        f_final.close()

    else:

        workbook = xlsxwriter.Workbook("Road_Data.xlsx")
        worksheet = workbook.add_worksheet("Scenario1")

        worksheet.write(0, 0, "Time")
        worksheet.write(0, 1, "Longitude")
        worksheet.write(0, 2, "Latitude")
        worksheet.write(0, 3, "Velocity")
        worksheet.write(0, 4, "RelativeVelocity")
        worksheet.write(0, 5, "Deceleration")
        worksheet.write(0, 6, "Distance")
        worksheet.write(0, 7, "Steep")
        worksheet.write(0, 8, "Angle")
        worksheet.write(0, 9, "DecelerationLead")
        worksheet.write(0, 10, "Gear")

        for idx, point in enumerate(road_points):
            worksheet.write(idx + 1, 0, str(idx))
            worksheet.write(idx + 1, 1, str(point.longitude).replace('.', ','))
            worksheet.write(idx + 1, 2, str(point.latitude).replace('.', ','))
            worksheet.write(idx + 1, 3, str(point.velocity).replace('.', ','))
            worksheet.write(idx + 1, 4, str(point.relative_velocity).replace('.', ','))
            worksheet.write(idx + 1, 5, str(point.deceleration).replace('.', ','))
            worksheet.write(idx + 1, 6, str(point.distance).replace('.', ','))
            worksheet.write(idx + 1, 7, str(point.steep))
            worksheet.write(idx + 1, 8, str(point.angle).replace('.', ','))
            worksheet.write(idx + 1, 9, str(point.deceleration_lead).replace('.', ','))
            worksheet.write(idx + 1, 10, str(point.gear))

        workbook.close()

    pass


f_name = 'TomTomApi_Modelovany_usek_cesty_response_1671693978046.json'

create_testing_data(for_python=False, source_file_name=f_name)
