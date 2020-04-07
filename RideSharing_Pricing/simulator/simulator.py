import numpy as np
from simulator.models.vehicle.vehicle_repository import VehicleRepository
from simulator.models.customer.customer_repository import CustomerRepository
from simulator.services.demand_generation_service import DemandGenerator
from simulator.services.routing_service import RoutingEngine
from common.time_utils import get_local_datetime
from config.settings import OFF_DURATION, PICKUP_DURATION
from simulator.settings import FLAGS
from logger import sim_logger
from logging import getLogger
from novelties import agent_codes
from random import randrange


class Simulator(object):
    def __init__(self, start_time, timestep):
        self.reset(start_time, timestep)
        sim_logger.setup_logging(self)
        self.logger = getLogger(__name__)
        self.demand_generator = DemandGenerator()
        self.routing_engine = RoutingEngine.create_engine()
        self.route_cache = {}
        self.current_dummyV = 0
        self.current_dqnV = 0

    def reset(self, start_time=None, timestep=None):
        if start_time is not None:
            self.__t = start_time
        if timestep is not None:
            self.__dt = timestep
        VehicleRepository.init()
        CustomerRepository.init()

    def populate_vehicle(self, vehicle_id, location):
        type = 0
        r = randrange(2)
        if r == 0 and self.current_dummyV < FLAGS.dummy_vehicles:
            type = agent_codes.dummy_agent
            self.current_dummyV += 1

        # If r = 1 or num of dummy agent satisfied
        elif self.current_dqnV < FLAGS.dqn_vehicles:
            type = agent_codes.dqn_agent
            self.current_dqnV += 1

        else:
            type = agent_codes.dummy_agent
            self.current_dummyV += 1


        VehicleRepository.populate(vehicle_id, location, type)

    def step(self):
        for customer in CustomerRepository.get_all():
            customer.step(self.__dt)
            if customer.is_arrived() or customer.is_disappeared():
                CustomerRepository.delete(customer.get_id())

        for vehicle in VehicleRepository.get_all():
            vehicle.step(self.__dt)
            # vehicle.print_vehicle()
            if vehicle.exit_market():
                score = ','.join(map(str, [self.get_current_time(), vehicle.get_id()] + vehicle.get_score()))
                sim_logger.log_score(score)
                VehicleRepository.delete(vehicle.get_id())

        self.__populate_new_customers()
        self.__update_time()
        if self.__t % 3600 == 0:
            # print("Elapsed : {}".format(get_local_datetime(self.__t)))
            self.logger.info("Elapsed : {}".format(get_local_datetime(self.__t)))

    def match_vehicles(self, commands, dqn_agent, dummy_agent):
        # print("M: ", commands)
        vehicle_list = []
        rejected_requests = []
        accepted_commands = {}
        reject_count = 0
        # Comamnd is a dictionary created in dummy_agent
        # print("########################################################")
        for command in commands:
            rejected_flag = 0
            # print(command["vehicle_id"], command["customer_id"])
            vehicle = VehicleRepository.get(command["vehicle_id"])
            # vehicle.state.status = status_codes.V_ASSIGNED
            if vehicle is None:
                self.logger.warning("Invalid Vehicle id")
                continue
            customer = CustomerRepository.get(command["customer_id"])
            if customer is None:
                self.logger.warning("Invalid Customer id")
                continue

            triptime = command["duration"]
            vid = command["vehicle_id"]
            # print("Maching: Vehicle " + vehicle.to_string() + " ---> " + customer.to_string())

            price_response = command["init_price"]
            if FLAGS.enable_pricing:
                # For DQN
                if vehicle.state.agent_type == agent_codes.dummy_agent:
                    price_response = dummy_agent.get_price_decision(vehicle, command["init_price"], customer.get_request())

                elif vehicle.state.agent_type == agent_codes.dqn_agent:
                    price_response = dqn_agent.get_price_decision(vehicle, command["init_price"], customer.get_request())
                # price_response = initial_price

                # print("Diff: ", (price_response-initial_price))

                # Now, customer needs to calculate reward and accpet or reject
                if customer.accpet_reject_ride(price_response, vehicle.state, triptime):   # If customer accepts
                    vehicle.head_for_customer(customer.get_origin(), triptime, customer.get_id(), command["distance"])
                    customer.wait_for_vehicle(triptime)
                    v = VehicleRepository.get(vid)
                    v.state.current_capacity += 1
                    accepted_commands[reject_count] = command
                    # print(command)
                    # print(accepted_commands)
                    reject_count += 1
                    # print(len(accepted_commands))

                else:           # base case: request drops
                    rejected_flag = 1
                    customer.go_to_nxt_timestep = 1
                    rejected_requests.append(customer.get_request())

            else:
                customer.accepted_price = command["init_price"]
                vehicle.head_for_customer(customer.get_origin(), triptime, customer.get_id(), command["distance"])
                customer.wait_for_vehicle(triptime)
                v = VehicleRepository.get(vid)
                v.state.current_capacity += 1
                accepted_commands = commands
                # vehicle.state.status = status_codes.V_ASSIGNED

            if FLAGS.enable_pooling:
                if vid not in vehicle_list and not rejected_flag:
                    vehicle_list.append(vid)

        if FLAGS.enable_pooling:
            for vi in vehicle_list:
                vehicle = VehicleRepository.get(vi)
                # vehicle.print_vehicle()
                vehicle.change_to_assigned()

        return rejected_requests, accepted_commands


    def dispatch_vehicles(self, commands):
        # print("D: ", commands)
        od_pairs = []
        vehicles = []
        # Comamnd is a dictionary created in dummy_agent
        for command in commands:
            vehicle = VehicleRepository.get(command["vehicle_id"])
            if vehicle is None:
                self.logger.warning("Invalid Vehicle id")
                continue

            if "offduty" in command:
                off_duration = self.sample_off_duration()   #Rand time to rest
                vehicle.take_rest(off_duration)
            elif "cache_key" in command:
                l, a = command["cache_key"]
                route, triptime = self.routing_engine.get_route_cache(l, a)
                vehicle.cruise(route, triptime)
            else:
                vehicles.append(vehicle)
                od_pairs.append((vehicle.get_location(), command["destination"]))

        routes = self.routing_engine.route(od_pairs)

        for vehicle, (route, triptime) in zip(vehicles, routes):
            if triptime == 0:
                continue
            vehicle.cruise(route, triptime)

    def __update_time(self):
        self.__t += self.__dt

    def __populate_new_customers(self):
        new_customers = self.demand_generator.generate(self.__t, self.__dt)
        CustomerRepository.update_customers(new_customers)

    def sample_off_duration(self):
        return np.random.randint(OFF_DURATION / 2, OFF_DURATION * 3 / 2)

    def sample_pickup_duration(self):
        return np.random.exponential(PICKUP_DURATION)

    def get_current_time(self):
        t = self.__t
        return t

    def get_new_requests(self):
        return CustomerRepository.get_new_requests()

    def get_vehicles_state(self):
        return VehicleRepository.get_states()

    def get_vehicles(self):
        return VehicleRepository.get_all()

    def get_customers(self):
        return CustomerRepository.get_all()
        # return [VehicleRepository.get(id) for id in v_ids]

    # def log_score(self):
    #     for vehicle in VehicleRepository.get_all():
    #         score = ','.join(map(str, [self.get_current_time(), vehicle.get_id()] + vehicle.get_score()))
    #         sim_logger.log_score(score)
