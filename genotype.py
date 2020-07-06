# import graph library
import networkx as nx
from random import randint,  uniform
from helper import *
logging = 1


class Genotype:
    def __init__(self):
        # Create an empty directed graph that allows multiple edges between nodes
        # Nodes contain information about a rigid body part:
        # dimensions, joint type, joint limits, recursive-limit
        # Connections contain the placement of a child relative to its parent part:
        # position, orientation, scale, reflection, terminal-only flag
        # A local group of neurons is also described in the node
        self.genotype = nx.MultiDiGraph()
        self.generateRandom()
        #self.generateStatic()


    def generateStatic(self):
        # Generates a static genotype with three consecutive for testing
        self.genotype.add_node( -1,
                                name             = 'root-node',
                                mass             = 1,
                                dimensions       = [.1, .1, .1],
                                base_position    = [ 1,  1,  1],
                                base_orientation = [ 0,  0,  0, 1],
                                joint_type       = randint(0,5),
                                rec_limit        = 2                )

        self.genotype.add_edge( -1, -1,
                                rel_position     = [ 0,  1,  0  ],
                                rel_orientation  = [ 0,  0,  0, 1],
                                scale            = [.5, .5, .5]     )

        self.genotype.add_edge( -1, -1,
                                rel_position     = [ 0,  1,  0  ],
                                rel_orientation  = [ 0,  0,  0, 1],
                                scale            = [.5, .5, .5]     )


    def generateRandom(self):
        # Generates a random genotype for testing
        #max_core_nodes    = randint(1,2)
        max_core_nodes    = 5
        # Generate root node first
        dimensions    = [ random3f ( 0.01, 0.1  ) ]
        mass          = 100
        x,y,z         = dimensions[0]
        self.genotype.add_node(   -1,
                                name             = 'root-node',
                                mass             = mass,
                                dimensions       = dimensions,
                                base_orientation = [ 0,  0,  0, 1],
                                joint_type       = 0,
                                rec_limit        = 2           )

        # Add at least one edge from the root node
        self.genotype.add_edge( -1,
                                randint( -1, max_core_nodes -1),
                                rel_position     = [ uniform( x, 2*x ), uniform( y, 2*y ), uniform(z, 2*z )  ],
                                rel_orientation  = [ 0,  0,  0, 1],
                                scale            = random3i(1, 2)     )
                                
        # Generate extra nodes
        
        for node in range( 0, max_core_nodes ):
            self.genotype.add_node( node,
                                    name             = str( node ),
                                    joint_type       = 0,
                                    rec_limit        = 1
                                    )

            for i in range (randint(1,5)):
                self.genotype.add_edge( node,
                                        randint( -1, max_core_nodes -1),
                                        rel_position     = [ uniform( x, 2*x ), uniform( y, 2*y ), uniform(z, 2*z )  ],
                                        rel_orientation  = [ 0,  0,  0, 1],
                                        scale            = random3i(1, 2)     )
                

        # for node in range(0,10):
        #     self.genotype.add_edge( randint(no, 9), randint(0, 9),
        #                         rel_position     =  uniform( x, 2*x ), uniform( y, 2*y ), uniform( z, 2*z )  ],
        #                         rel_orientation  = [ 0,  0,  0, 1],
        #                         scale            = [1, 1, 1]     )


        # self.genotype.add_edge( -1, -1,
        #                         #rel_position     = [ uniform( x, 2*x ), uniform( y, 2*y ), uniform( z, 2*z )  ],
        #                         rel_position     = [ 0.0,  10.0,  0.0],
        #                         rel_orientation  = [ 0,  0,  0, 1],
        #                         scale            = [1, 1, 1]     )
                                

    def showInfo(self):
        # Shows nodes edges and data of the genotypes' directed graph
        print '*Genotype Node List: ', self.genotype.nodes
        print '*Genotype Node Data: '
        d = self.genotype.nodes.data()
        for node in d:
            print 'Node:', node[0] 
            for key,value  in node[1].iteritems():
                print ' ', key, value
                
        print '*Genotype Edge List: ', self.genotype.edges
        print '*Genotype Edge Data: '
        d = self.genotype.edges.data() 
        for edge in d:
            print 'Parent Node:', edge[0] 
            print 'Child Node:',  edge[1] 
            for key, value  in    edge[2].iteritems():
                print ' ', key, value

    def getData(self):
        n = self.genotype.nodes.data()
        e = self.genotype.edges.data() 
        return n, e
        