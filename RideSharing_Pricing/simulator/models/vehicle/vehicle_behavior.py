import simulator.models.customer.customer_repository
from common import geoutils
from simulator.settings import FLAGS

class VehicleBehavior(object):
    available = True

    def step(self, vehicle, timestep):
        pass


class Idle(VehicleBehavior):
    pass


class Cruising(VehicleBehavior):
    # Updated remaining time to destination, if arrived states changes to parking
    def step(self, vehicle, timestep):
        arrived = vehicle.update_time_to_destination(timestep)
        if arrived:
            vehicle.park()
            return

        self.drive(vehicle, timestep)


    def drive(self, vehicle, timestep):
        route = vehicle.get_route()      # Sequence of (lon, lat)
        speed = vehicle.get_speed()
        dist_left = timestep * speed    # Remaining Distance
        rlats, rlons = zip(*([vehicle.get_location()] + route)) # New vehicle location after driving this route
        step_dist = geoutils.great_circle_distance(rlats[:-1], rlons[:-1], rlats[1:], rlons[1:])    # Get distcnace in meters
        for i, d in enumerate(step_dist):
            if dist_left < d:
                bearing = geoutils.bearing(rlats[i], rlons[i], rlats[i + 1], rlons[i + 1])      # Calculate angle of motion
                next_location = geoutils.end_location(rlats[i], rlons[i], dist_left, bearing)   # Calculate nxt location
                vehicle.update_location(next_location, route[i + 1:])           # Updating location based on route's nxt (lon, lat)
                return
            dist_left -= d

        if len(route) > 0:
            vehicle.update_location(route[-1], [])  # Go the last step


class Occupied(VehicleBehavior):
    available = False
    # Updated remaining time to destination, if arrived customer gets off
    def step(self, vehicle, timestep):
        arrived = vehicle.update_time_to_destination(timestep)
        if arrived:
            # customer = vehicle.dropoff()
            # customer.get_off()
            vehicle.dropoff()
            # env.models.customer.customer_repository.CustomerRepository.delete(customer.get_id())


class Assigned(VehicleBehavior):
    available = False
    # Updated remaining time to destination, if arrived, update customer ID and picks him up
    def step(self, vehicle, timestep):
        arrived = vehicle.update_time_to_destination(timestep)
        if arrived:
            if FLAGS.enable_pooling:
                # print("Assigned, pooling!")
                ids = vehicle.get_customers_ids()
                # print("Arrived: Vehicle Info", vehicle.to_string())
                # print("Customer ids:", ids)
                for i in range(len(ids)):
                    customer = simulator.models.customer.customer_repository.CustomerRepository.get(ids[i])
                    # print("Customer Info:", customer.to_string())
                    customer.ride_on()
                    vehicle.update_customers(customer)
                    if i == len(ids) - 1:
                        vehicle.pickup(customer)
            else:
                # print("Assigned, not pooling!")
                customer = simulator.models.customer.customer_repository.CustomerRepository.get(
                vehicle.get_assigned_customer_id())
                vehicle.pickup(customer)

class OffDuty(VehicleBehavior):
    available = False
    # Updated remaining time to destination, if returned state changes to parking
    def step(self, vehicle, timestep):
        returned = vehicle.update_time_to_destination(timestep)
        if returned:
            vehicle.park()
