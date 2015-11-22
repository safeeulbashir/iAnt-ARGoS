from lxml import etree
import numpy as np
import copy
import csv
import argparse


LINUX_CONTROLLER_LIB = "build/controllers/libiAnt_controller.so"
MAC_CONTROLLER_LIB = "build/controllers/libiAnt_controller.dylib"
LINUX_LOOP_LIB = "build/loop_functions/libiAnt_loop_functions.so"
MAC_LOOP_LIB = "build/loop_functions/libiAnt_loop_functions.dylib"
ARGOS_XML_DEFAULT = '''<?xml version="1.0"?>
<argos-configuration>
<framework>
<!--
    <experiment length           = "0"
                ticks_per_second = "16"
                random_seed      = "1337"/>
-->
    <experiment ticks_per_second = "16"
                random_seed      = "1337"/>
</framework>
<controllers>
    <iAnt_controller id      = "iAnt_c"
                     library = "build/source/libiAnt_controller">
        <actuators>
            <differential_steering implementation = "default"/>
        </actuators>
        <sensors>
            <footbot_proximity    implementation = "default"
                                  show_rays      = "false"/>
            <positioning          implementation = "default"/>
            <footbot_motor_ground implementation = "rot_z_only"/>
        </sensors>
        <!-- un-evolvable parameters -->
        <!-- remember: a footbot's radius = 8.5 cm / 0.085 m -->
        <params>
            <iAnt_params searchStepSize          = "0.175"
                         distanceTolerance       = "0.01"
                         robotForwardSpeed       = "16.0"
                         robotRotationSpeed      = "13.3"
                         angleToleranceInDegrees = "15.0"/>
            </params>
        </iAnt_controller>
    </controllers>
    <!-- LOOP FUNCTIONS -->
    <loop_functions library = "build/source/libiAnt_loop_functions"
                    label   = "iAnt_loop_functions">
        <!-- evolvable parameters -->
        <CPFA       ProbabilityOfSwitchingToSearching = "0.2041456252336502"
                    ProbabilityOfReturningToNest      = "0.0009405957534909248"
                    UninformedSearchVariation         = "12.996437"
                    RateOfInformedSearchDecay         = "0.2497878670692444"
                    RateOfSiteFidelity                = "2.5117506980896"
                    RateOfLayingPheromone             = "2.24421238899231"
                    RateOfPheromoneDecay              = "0.03821808844804764"/>
        <!-- un-evolvable environment variables -->
        <simulation MaxSimCounter        = "1"
                    MaxSimTime           = "3600"
                    VariableSeed         = "1"
                    OutputData           = "1"
                    ResourceDensityDelay = "4"
                    DrawDensityRate      = "8"
                    DrawTrails           = "1"
                    DrawTargetRays       = "1"
                    NestPosition         = "0.0, 0.0"
                    NestRadius           = "0.25"
                    NestElevation        = "0.01"
                    FoodRadius           = "0.05"
                    FoodDistribution     = "2"/>
        <!-- un-evolvable food distribution parameters -->
        <_0_FoodDistribution_Random   FoodItemCount    = "256"/>
        <_1_FoodDistribution_Cluster  NumberOfClusters = "4"
                                      ClusterWidthX    = "8"
                                      ClusterLengthY   = "8"/>
        <_2_FoodDistribution_PowerLaw PowerRank        = "5"/>
    </loop_functions>
    <!-- ARENA -->
    <arena size="20.0, 20.0, 1.0" center="0.0, 0.0, 0.0">
        <floor id="floor" source="loop_functions" pixels_per_meter="10"/>
        <distribute>
            <!--
            <position method = "uniform"
                      min    = "-1, -1, 0"
                      max    = "1, 1, 0"/>
            <orientation method="gaussian" mean="0, 0, 0" std_dev="360, 0, 0"/>
            -->
            <position method="grid"
                      center="0.0, 0.0, 0.0"
                      distances="0.2, 0.2, 0.0"
                      layout="3, 4, 1" />
            <orientation method="constant" values="0.0, 0.0, 0.0" />
            <entity quantity="12" max_trials="500">
                <foot-bot id="fb_"><controller config="iAnt_c"/></foot-bot>
            </entity>
        </distribute>
    </arena>
    <!-- PHYSICS ENGINE(S) -->
    <physics_engines><dynamics2d id="dyn2d"/></physics_engines>
    <!-- MEDIA -->
    <media><led id="leds"/></media>
    <!-- VISUALIZATION -->
     <!-- <visualization>
        <qt-opengl>
            <camera>
                <placement idx="0" position="  0, 0, 10" look_at="0, 0, 0" lens_focal_length="10"/>
                <placement idx="1" position="  0, 0, 10" look_at="0, 0, 0" lens_focal_length="45"/>
                <placement idx="2" position="  0, 0, 10" look_at="0, 0, 0" lens_focal_length="120"/>
                <placement idx="3" position="-30, 0, 10" look_at="0, 0, 0" lens_focal_length="180"/>
                <placement idx="4" position="-30, 0, 10" look_at="0, 0, 0" lens_focal_length="240"/>
            </camera>
            <user_functions label="iAnt_qt_user_functions"/>
        </qt-opengl>
    </visualization>-->
</argos-configuration>
'''


CPFA_LIMITS = {
    "RateOfLayingPheromone": (0, 20),
    "RateOfPheromoneDecay": (0, 1),
    "ProbabilityOfSwitchingToSearching": (0, 1),
    "RateOfSiteFidelity": (0, 20),
    "RateOfInformedSearchDecay": (0, 1),
    "ProbabilityOfReturningToNest": (0, 1),
    "UninformedSearchVariation": (0, 359)
}


def default_argos_xml(robots, time, system="linux"):
    xml = etree.fromstring(ARGOS_XML_DEFAULT)
    xml.find("arena").find("distribute").find(
        "entity").attrib["quantity"] = str(robots)
    exp_att = xml.find("loop_functions").find("simulation").attrib
    exp_att.update({"MaxSimTime": str(time)})

    if system == "linux":
        return xml
    elif system == "darwin":
        xml.find("controllers").find(
            "iAnt_controller").attrib["library"] = MAC_CONTROLLER_LIB
        xml.find("loop_functions").attrib["library"] = MAC_LOOP_LIB
        return xml
    else:
        return None


def uniform_rand_argos_xml(robots, time, system="linux"):
    xml = default_argos_xml(robots, time, system)
    cpfa = {}
    for key in CPFA_LIMITS:
        cpfa[key] = str(np.random.uniform(CPFA_LIMITS[key][0], CPFA_LIMITS[key][1]))
    set_cpfa(xml, cpfa)
    return xml


def get_cpfa(argos_xml):
    return argos_xml.find("loop_functions").find(
        "CPFA").attrib


def set_cpfa(argos_xml, cpfa_update):
    attrib = argos_xml.find("loop_functions").find(
        "CPFA").attrib
    attrib.update(cpfa_update)


def set_seed(argos_xml, seed):
    attrib = argos_xml.find("framework").find("experiment").attrib
    attrib.update({"random_seed": str(int(seed))})


def mutate_cpfa(argos_xml, probability):
    cpfa = get_cpfa(argos_xml)
    for key in CPFA_LIMITS:
        if np.random.uniform() > probability:
            val = float(cpfa[key])
            val += np.random.normal(0, 0.05)
            if val > CPFA_LIMITS[key][1]:
                val = CPFA_LIMITS[key][1]
            elif val < CPFA_LIMITS[key][0]:
                val = CPFA_LIMITS[key][0]
            cpfa[key] = str(val)
    set_cpfa(argos_xml, cpfa)


def uniform_crossover(p1_xml, p2_xml, system="linux"):
    p1_cpfa = copy.deepcopy(get_cpfa(p1_xml))
    # Initialize child to parent 2 cpfa
    child_cpfa = copy.deepcopy(get_cpfa(p2_xml))
    from_p1 = False
    for key in CPFA_LIMITS:
        if from_p1:
            child_cpfa[key] = p1_cpfa[key]
        from_p1 = not from_p1
    parent_time = p1_xml.find("loop_functions").find("simulation").attrib["MaxSimTime"]
    parent_robots = p1_xml.find("arena").find("distribute").find(
        "entity").attrib["quantity"]
    child = default_argos_xml(system=system, time=parent_time, robots=parent_robots)
    set_cpfa(child, child_cpfa)
    return child

def read_pop_from_csv(filename):
    with open(filename, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        return list(reader)

def xml_string_cpfa_chunk(cpfa):
    xml = default_argos_xml(6, 3600)
    cpfa_xml = xml.find("loop_functions").find("CPFA")
    cpfa_xml.attrib.update(cpfa)
    return etree.tostring(cpfa_xml)

def create_argos_from_cpfa(cpfa, robots, length, system):
    xml = default_argos_xml(robots, length, system)
    cpfa_xml = xml.find("loop_functions").find("CPFA")
    cpfa_xml.attrib.update(cpfa)
    return etree.tostring(xml, pretty_print=True)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='CPFA XML printer')
    parser.add_argument('-f', '--gen_file', action='store', dest='gen_file', required=True)
    parser.add_argument('-a', '--all', action='store', dest='all')
    parser.add_argument('-s', '--searchradius', action='store', dest='sradius')
    parser.add_argument('-r', '--robots', action="store", dest="robots")
    parser.add_argument('-l', '--length', action='store', dest='length')
    parser.add_argument('-c', '--create', action="store_true", dest="create")
    parser.add_argument('--system', action='store', dest='system')

    args = parser.parse_args()

    gen_file = args.gen_file
    robots='10'
    length='300'
    sradius='0.01'
    system='linux'

    pop = read_pop_from_csv(gen_file)
    
    if args.create:
        if args.length:
            length=args.length
        if args.sradius:
            sradius=args.sradius
        if args.robots:
            robots=args.robots
        if args.system:
            system = args.system
        print create_argos_from_cpfa(pop[0], robots, length, system)
    elif args.all:
        for p in pop:
            print "Fitness:", p["fitness"]
            print xml_string_cpfa_chunk(p)
    else:
        print "Fitness:", pop[0]["fitness"]
        print xml_string_cpfa_chunk(pop[0])