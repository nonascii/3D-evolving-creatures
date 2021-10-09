# import graph library
import networkx as nx
from random import randint,  uniform
from  genotype import *
logging = 1


class Phenotype:
    # Represents a hierarchical order of 3D parts
    def __init__(self, p):

        # Initialize createMultiBody argument arrays
        self.link_Masses  				   = []
        self.linkCollisionShapeIndices     = []
        self.linkVisualShapeIndices        = []
        self.linkPositions    			   = []
        self.linkOrientations 			   = []
        self.linkInertialFramePositions    = []
        self.linkInertialFrameOrientations = []
        self.parent_indices                = []
        self.jointTypes                    = []
        self.axis                          = []

        # Create seperate array to store part dimensions
        self.linkDimensions                = []
        
        # Create a list of bodypart ids to represent the body
        self.body = []

        # Create a new genotype to be based upon
        self.g = Genotype()

        # Create a new directed multigraph to represent the phenotype
        # after parsing the genotype graph
        self.p = nx.MultiDiGraph()

        # # Traverse the graph by starting at the root node and following the edges
        # edge_list  = list( nx.edge_dfs( self.g.genotype, -1 ) )
        # node_index = 0
        

        # for di_edge in edge_list:
        #     parent_node      = self.g.genotype.nodes[ di_edge[0] ]
        #     child_node       = self.g.genotype.nodes[ di_edge[1] ]
        #     recursive_limit  = parent_node['rec_limit']
            
        #     print parent_node
        #     print child_node
        #     self.p.add_node( parent_node )
        #     print self.p.nodes( data = True )
            
        
        # raw_input('pause')



        nodes_data, edges_data = self.g.getData()

        # create root node from genotype
        #log( '\n {0}'.format(  self.g.genotype.nodes(data = True)) )
        log( '\nNode List: {0}'.format(  self.g.genotype.nodes()))
        log( 'Edge List: {0}'.format( self.g.genotype.edges() ) )
        
        # Start at root node
        root_node            = self.g.genotype.nodes[ -1 ] 
        neighbors            = list( self.g.genotype.adj[ -1 ] ) 
        root_node_dimensions = root_node[ 'dimensions' ]
        self.root_node_dimensions = root_node_dimensions
        x, y, z              = root_node_dimensions[0]
        root_node_id         = p.createCollisionShape( p.GEOM_BOX, halfExtents = [ x,y,z ] )
        root_node_vid        = -1
        root_node_pos        = random3f ( -1, 1  ) 
        root_node_ori        = root_node[ 'base_orientation' ]
        
            
        log( '\nParsing root node' )
        log( 'Neighbor nodes: {0}'.format( neighbors ) )
        
        # Iterate root nodes' neighbors
        for neighbor in neighbors:
            
            parent_node = root_node
            child_node  = self.g.genotype.nodes[ neighbor ]
            edge        = self.g.genotype[ -1 ][ neighbor ]
            print (edge[0]['scale'])
            iter_limit = parent_node[ 'rec_limit' ]
            log( '\nParsing edge: {0} --> {1}'.format( -1, neighbor ) )
            
            current_node_index   = 0

            for i in range( 0, iter_limit ):                
                self.dimensions = parent_node[ 'dimensions' ]
                bx, by, bz      = self.dimensions[0]
                sx, sy, sz      = edge[0]['scale']
                rx, ry, rz      = edge[0]['rel_position']
                rx += current_node_index * .1
                ry += current_node_index * .1
                rz += current_node_index * .1
                linkId          = p.createCollisionShape(   p.GEOM_BOX, 
                                                            halfExtents = [bx*sx, by*sy, bz*sz] )
                

                self.linkDimensions.append(                ( bx*sx,  by*sy,  bz*sz  ))
                self.link_Masses.append(                     100   )                                   
                self.linkCollisionShapeIndices.append(       linkId                  )
                self.linkVisualShapeIndices.append(          -1                      ) 
                self.linkPositions.append(                   [rx,ry,rz]              )
                self.linkOrientations.append(                [0,0,0,1]               )
                self.linkInertialFramePositions.append(      [0,0,0  ]               )
                self.linkInertialFrameOrientations.append(   [0,0,0,1]               )
                self.parent_indices.append(                  current_node_index      )
                self.jointTypes.append(                      0                       )
                self.axis.append(                            ( randint(0,1), randint(0,1), randint(0,1) ) ) 
                #self.axis.append(                            (1.0, 1.0, 1.0)         )

                current_node_index   += 1
                neighbor_list            = list( self.g.genotype.adj[ neighbor ] ) 

                print ('Child_nei:', neighbor_list)

                #print root_node['mass']
                # print bx*sx,  by*sy,  bz*sz
                # print bx*sx * by*sy * bz*sz
                # print linkId
                # print -1                     
                # print edge[2]['rel_position']
                # print [0,0,0,1]              
                # print [0,0,0  ]              
                # print self.linkOrientations            
                # print i                      
                # print 0    
                # print self.axis                  
                                        
            
        # Create the body part based on the generated childs
        self.bodyPartId = p.createMultiBody( 
                                    root_node['mass'],
                                    root_node_id,
                                    root_node_vid,
                                    random3f ( -1.0, 1.0  ),
                                    root_node_ori,
                                    linkMasses                    = self.link_Masses,
                                    linkCollisionShapeIndices     = self.linkCollisionShapeIndices,
                                    linkVisualShapeIndices        = self.linkVisualShapeIndices,
                                    linkPositions                 = self.linkPositions,
                                    linkOrientations              = self.linkOrientations,
                                    linkInertialFramePositions    = self.linkInertialFramePositions,
                                    linkInertialFrameOrientations = self.linkInertialFrameOrientations,
                                    linkParentIndices             = self.parent_indices,
                                    linkJointTypes                = self.jointTypes,
                                    linkJointAxis                 = self.axis      )
        

        self.body.append(   self.bodyPartId   )
        self.link_Masses  				   = []
        self.linkCollisionShapeIndices     = []
        self.linkVisualShapeIndices        = []
        self.linkPositions    			   = []
        self.linkOrientations 			   = []
        self.linkInertialFramePositions    = []
        self.linkInertialFrameOrientations = []
        self.parent_indices                = []
        self.jointTypes                    = []
        self.axis                          = []


        # Change various dynamics parameters
        p.changeDynamics(           self.bodyPartId,      -1, 
                                    spinningFriction = 0.001,
                                    rollingFriction  = 0.001, 
                                    linearDamping    = 0.0     )
            

    def getDimensions(self):
        return self.linkDimensions
    
    def getRootDimensions(self):
        return self.root_node_dimensions
