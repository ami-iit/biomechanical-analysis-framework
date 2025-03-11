import pandas as pd

def getSensorToLinkMap(nodeID, attachedLink, nodes):
    """ getSensorToLinkMap creates a table for mapping nodes ID and links where the nodes are attached to."""
    
    rows = []
    final_nodes = []
    
    for i,node in enumerate(nodes):
        if node is not None:
            node['attachedLink'] = attachedLink[i]
            
            row = {
                'nodeID': nodeID[i],
                'attachedLink': attachedLink[i],
                'data' : node
            }
        
            rows.append(row)
            final_nodes.append(node)     
    sensor2linkMap = pd.DataFrame(rows)
    
    return sensor2linkMap, final_nodes