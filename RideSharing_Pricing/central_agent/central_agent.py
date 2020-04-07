from novelties import status_codes
from simulator.settings import FLAGS
from simulator.models.vehicle.vehicle_repository import VehicleRepository
from simulator.models.customer.customer_repository import CustomerRepository
from common.geoutils import great_circle_distance
from novelties.pricing.price_calculator import calculate_price

class Central_Agent(object):

    def __init__(self, matching_policy):
        self.matching_policy = matching_policy

    def get_match_commands(self, current_time, vehicles, requests):
        matching_commands = []
        if len(requests) > 0:
            if FLAGS.enable_pooling:
                matching_commands = self.matching_policy.match_RS(current_time, vehicles, requests)
            else:
                matching_commands = self.matching_policy.match(current_time, vehicles, requests)


        updated_commands = self.init_price(matching_commands)
            # V = defaultdict(list)
            # for command in matching_commands:
            #     # print(command)
            #     V[command["vehicle_id"]].append(command["customer_id"])
            #
            # for k in V.keys():
            #     print(k, "Customers :", V[k])

        vehicles = self.update_vehicles(vehicles, matching_commands)

        return updated_commands, vehicles


    def update_vehicles(self, vehicles, commands):
        vehicle_ids = [command["vehicle_id"] for command in commands]
        vehicles.loc[vehicle_ids, "status"] = status_codes.V_ASSIGNED
        return vehicles


    def init_price(self, match_commands):
        m_commands = []
        for c in match_commands:
            vehicle = VehicleRepository.get(c["vehicle_id"])
            # vehicle.state.status = status_codes.V_ASSIGNED
            if vehicle is None:
                print("Invalid Vehicle id")
                continue
            customer = CustomerRepository.get(c["customer_id"])
            if customer is None:
                print("Invalid Customer id")
                continue

            triptime = c["duration"]

            # if FLAGS.enable_pricing:
            dist_for_pickup = c["distance"]
            dist_till_dropoff = great_circle_distance(customer.get_origin()[0], customer.get_origin()[1],
                                          customer.get_destination()[0], customer.get_destination()[1])
            total_trip_dist = dist_for_pickup + dist_till_dropoff
            [travel_price, wait_price] = vehicle.get_price_rates()
            # v_cap = vehicle.state.current_capacity
            initial_price = calculate_price(total_trip_dist, triptime, vehicle.state.mileage, travel_price,
                                wait_price, vehicle.state.gas_price, vehicle.state.driver_base_per_trip)
            c["init_price"] = initial_price
            m_commands.append(c)
            # print(c)

        return m_commands