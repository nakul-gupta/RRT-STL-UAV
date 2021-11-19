%RRT
%import java.util.*;

classdef RRT_Node < handle
    properties
        state
        parent
        children
        hasParent
    end
    methods
        function node = RRT_Node(coords, parentNode)
            node.state = coords;
            node.parent = parentNode;
            node.children = [];
            node.hasParent = 1;
        end
    end
end



