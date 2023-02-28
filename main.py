import json
import random
import xlsxwriter


class RoadPointTomTom:
    longitude: float
    latitude: float
    speed_limit: float

    def __init__(self, longitude, latitude, speed_limit):
        self.longitude = longitude
        self.latitude = latitude
        self.speed_limit = speed_limit


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

    def __init__(self, longitude, latitude, deceleration, velocity, relative_velocity, distance, steep, angle, deceleration_lead, gear):
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


def set_road_points_tomtom(tomtom_json):
    # Speed limits
    # 90 km/h -> 25 m/s
    # 70 km/h -> 19.4444 m/s
    # 50 km/h -> 13.8889 m/s
    # 30 km/h -> 8.3333 m/s
    __tomtom_points = []

    for __point in tomtom_json['routes'][0]['legs'][0]['points']:
        tomtom_point = RoadPointTomTom(longitude=__point['longitude'], latitude=__point['latitude'], speed_limit=0)

        if (
                48.98994 >= __point['longitude'] >= 48.97638 and 21.94134 >= __point['latitude'] >= 21.94089
        ) or (
                48.96604 >= __point['longitude'] >= 48.95713 and 21.94681 >= __point['latitude'] >= 21.94549
        ) or (
                48.95438 >= __point['longitude'] >= 48.9429 and 21.9451 >= __point['latitude'] >= 21.93883
        ):
            tomtom_point.speed_limit = 25.0

        elif (
                48.97638 >= __point['longitude'] >= 48.96725 and 21.94089 >= __point['latitude'] >= 21.94674
        ) or (
                48.95713 >= __point['longitude'] >= 48.95438 and 21.94549 >= __point['latitude'] >= 21.9451
        ):
            tomtom_point.speed_limit = 19.4444

        else:
            tomtom_point.speed_limit = 13.8889

        __tomtom_points.append(tomtom_point)

    return __tomtom_points


def create_testing_data(for_python: bool, source_file_name):

    f_coordinates = open(source_file_name, mode='r')

    coordinates = json.load(f_coordinates)

    road_points = []

    tomtom_points = set_road_points_tomtom(coordinates)

    f_coordinates.close()

    velocity_delta = 3.6  # +- 3.6 m/s

    # in m/s^2 ... skoda octavia accelerates 0-100 km/h in 6.8s -> max acceleration is 4.08 m/s^2
    max_acceleration = 4.08
    acceleration_delta = 0.5  # +- 0.5 m/s^2

    # https://www.skoda-storyboard.com/en/skoda-world/innovation-and-technology/how-do-brakes-learn-how-to-brake/
    # decelerates 100-0km/h in 33-34m https://www.toppr.com/guides/physics-formulas/deceleration-formula/ using
    # formula a = (v^2 - u^2)/2s
    max_deceleration = 9.32835

    # 90 - 70km/h ; 70 - 50km/h ; 50 - 30 km/h; 100-0km/h ... first 3 are stopping on 50m; last one is max deceleration
    deceleration_values = [2.46915, 1.85183, 1.23458, max_deceleration]
    normal_deceleration_delta = 0.4  # +- 0.4 m/s^2

    max_distance = 150  # in m ... max detectable distance for obstacle
    distance_delta = 5  # in m

    max_relative_velocity = 8.3333  # difference in other vehicle's velocity is about 30 km/h
    relative_velocity_delta = 0.5  # +- 5 m/s

    deceleration_lead = 0

    for idx, point in enumerate(tomtom_points):

        # DISTANCE
        if idx > 0:
            distance = random.uniform(road_points[idx - 1].distance - distance_delta, max_distance)
        else:
            distance = random.uniform(0, max_distance)

        # VELOCITY
        velocity = random.uniform(point.speed_limit - velocity_delta, velocity_delta + point.speed_limit)

        # GEAR
        # 110 km/h -> 30.5556 m/s - 6
        # 90 km/h -> 25 m/s - 5
        # 70 km/h -> 19.4444 m/s - 4
        # 50 km/h -> 13.8889 m/s - 3
        # 30 km/h -> 8.3333 m/s - 2
        # 15 km/h -> 4.1667 m/s - 1
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


        # RELATIVE VELOCITY
        if idx > 0:
            # speed up and got closer
            # OR
            # speed down but obstacle is closer
            if (velocity > road_points[idx - 1].velocity and distance < road_points[idx - 1].distance) or (
                    velocity < road_points[idx - 1].velocity and distance < road_points[idx - 1].distance):

                relative_velocity = random.uniform(road_points[idx - 1].relative_velocity,
                                                   road_points[idx - 1].relative_velocity + relative_velocity_delta)

            # speed up but so does leading car/obstacle
            # OR
            # speed down and obstacle is farther
            elif (velocity > road_points[idx - 1].velocity and distance > road_points[idx - 1].distance) or (
                    velocity < road_points[idx - 1].velocity and distance > road_points[idx - 1].distance):

                relative_velocity = random.uniform(road_points[idx - 1].relative_velocity - relative_velocity_delta,
                                                   road_points[idx - 1].relative_velocity)

            # speed and distance is constant
            elif abs(velocity - road_points[idx - 1].velocity) < 0.0000000001 and abs(
                    distance - road_points[idx - 1].distance) < 0.0000000001:

                relative_velocity = 0.0000000001

            else:
                relative_velocity = random.uniform(max_relative_velocity - relative_velocity_delta,
                                                   max_relative_velocity + relative_velocity_delta)
        else:
            relative_velocity = 0.0000000001

        # DECELERATION
        if 25 >= point.speed_limit <= 19.4444:
            indx_decel = 0
        elif 19.4444 >= point.speed_limit <= 13.8889:
            indx_decel = 1
        elif 13.8889 >= point.speed_limit <= 8.3333:
            indx_decel = 2
        else:
            indx_decel = 3

        # relative velocity = v_lead - v_ours
        # if v_rel > 0  ... leading vehicle is moving faster
        # if v_rel < 0  ... our vehicle is moving faster
        # if v_rel == 0 ... constant speed of both vehicles
        if idx > 0:
            if distance < road_points[idx - 1].distance:
                # accelerating
                if velocity > road_points[idx - 1].velocity:
                    deceleration = - random.uniform(acceleration_delta, max_acceleration)

                # decelerating
                else:
                    deceleration = random.uniform(normal_deceleration_delta, deceleration_values[indx_decel])

            else:
                # accelerating
                if velocity > road_points[idx - 1].velocity:
                    deceleration = - random.uniform(0, max_acceleration)

                # decelerating
                else:
                    deceleration = random.uniform(0, deceleration_values[indx_decel])

        else:
            deceleration = random.uniform(-deceleration_values[indx_decel], deceleration_values[indx_decel])

        # LEADING CAR DECELERATION - for ISO 15623:2013 verification
        # if v_rel_prev > v_rel_now -> v_lead > v_our -> dec_lead < dec_our
        # if v_rel_prev < v_rel_now -> v_lead < v_our -> dec_lead > dec_our
        # if v_rel_prev ~= v_rel_now -> v_lead ~= v_our -> dec_lead ~= dec_our
        if idx > 0:
            velocity_lead = relative_velocity + velocity

            deceleration_diff = velocity / velocity_lead

            deceleration_lead = deceleration * deceleration_diff

        # OTHER VALUES
        steep = 1  # 1 is for UPHILL; -1 is for DOWNHILL
        angle = 0

        road_point = RoadPointFinal(longitude=point.longitude,
                                    latitude=point.latitude,
                                    relative_velocity=relative_velocity,
                                    velocity=velocity,
                                    distance=distance,
                                    deceleration=deceleration,
                                    steep=steep,
                                    angle=angle,
                                    deceleration_lead=deceleration_lead,
                                    gear=gear)

        road_points.append(road_point)
        print(idx)

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
            worksheet.write(idx + 1, 1, str(point.longitude))
            worksheet.write(idx + 1, 2, str(point.latitude))
            worksheet.write(idx + 1, 3, str(point.velocity))
            worksheet.write(idx + 1, 4, str(point.relative_velocity))
            worksheet.write(idx + 1, 5, str(point.deceleration))
            worksheet.write(idx + 1, 6, str(point.distance))
            worksheet.write(idx + 1, 7, str(point.steep))
            worksheet.write(idx + 1, 8, str(point.angle))
            worksheet.write(idx + 1, 9, str(point.deceleration_lead))
            worksheet.write(idx + 1, 10, str(point.gear))

        workbook.close()

    pass


f_name = 'TomTomApi_Modelovany_usek_cesty_response_1671693978046.json'

create_testing_data(for_python=True, source_file_name=f_name)
