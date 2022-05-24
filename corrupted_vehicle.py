import carla
import time

class CorruptedVehicle(object):
    def __init__(self, args, vehicle):
        self.vehicle = vehicle
        self.id = vehicle.id
        self.bounding_box = vehicle.bounding_box
        self.args = args

        self.flip_transform = False
        self.flip_time = time.monotonic()

    def get_world(self):
        return self.vehicle.get_world()

    def get_velocity(self):
        return self.vehicle.get_velocity()

    def get_speed_limit(self):
        return self.vehicle.get_speed_limit()

    def get_traffic_light(self):
        return self.vehicle.get_traffic_light()

    def get_control(self):
        return self.vehicle.get_control()

    # this isn't actually used by auto agent so no point hacking it
    def is_at_traffic_light(self):
        return self.vehicle.is_at_traffic_light()

    # possibly want to hack        
    def set_zero_location(self, val):
        self.args.attack_zero_location = val

    def get_location(self):
        if self.args.attack_zero_location:
            return carla.Location()
        return self.vehicle.get_location()

    def get_transform(self):
        if self.args.attack_swerve:
            out = self.vehicle.get_transform()
            extra = 10 # degrees
            if self.flip_transform:
                out.rotation.yaw += extra
            else:
                out.rotation.yaw -= extra
            if time.monotonic() > self.flip_time + 3: # every 3 seconds add / subtract 10 degrees
                self.flip_transform = not self.flip_transform
                self.flip_time = time.monotonic()
            return out
        else:
            return self.vehicle.get_transform()

    def get_speed_limit(self):
        return self.vehicle.get_speed_limit()

    # hacked
    def get_traffic_light_state(self):
        if self.args.attack_ignore_lights:
            return carla.TrafficLightState.Green
        else:
            return self.vehicle.get_traffic_light_state()
