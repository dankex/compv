#ifndef BK_CUT_GRAPH_T
#define BK_CUT_GRAPH_T

#include "bit_vector.h"

/* An implementation of the algorithm described in:

    An Experimental Comparison of Min-Cut/Max-Flow Algorithms for
    Energy Minimization in Vision

    Yuri Boykov and Vladimir Kolmogorov

   Implementation by Walter Bell (wbell@cs.cornell.edu) 2001.

   This code implements the algorithm for finding the min-cut/max-flow
   on a graph as described in the paper. It is memory and efficiency
   optimized for vision applications in which neighbor nodes have
   connections to each other and nodes have connections to both the
   source and the sink. It will work on arbitrary graphs however, but
   the memory allocation pattern will not be as ideal as for vision
   applications. This code implements a few additional optimizations
   not described in the paper such as not finding a parent for an
   orphaned sink node during the adoption phase, and these are
   documented in the code.  
*/
class BKCutGraph
{
public:
  /* The types for flows and link capacities. By making these smaller
     types, one can lessen the memory usage for large graphs.  
  */
  typedef unsigned int flowtype; 
  typedef unsigned int tcaptype; 
  typedef unsigned int ncaptype; 

  /* Enumeration for the terminal to add weight to. */
  typedef enum {SINK = 0, SOURCE = 1} termtype; 

  /* Construct a graph of n nodes. This will initialize the graph with
     no edges, and arcs must be added via add_arc(), add_edge(), and
     add_weight() before calling maxflow() to get the flow across the
     graph.
  */
  BKCutGraph(int n);
  virtual ~BKCutGraph();

  /* Reset the graph. This will return you to the state at
   construction time.
  */
  void reset();

  /* Add an arc between node 'from' and node 'to' with the given
     capacity. Additional calls to this method with the same nodes [or
     reversed pairs] are not allowed.
  */
  void add_arc(int from, int to, ncaptype cap);
  
  /* Add an edge [bidirectional] between node 'to' and node 'from'
     with capacity cap. A reverse edge will also be created with
     capacity rev_cap. Additional calls to this method with the same
     to and from [or reversed] pairs are not allowed.  
  */
  void add_edge(int from, int to, ncaptype cap, ncaptype cap_rev);

  /* Add some weight from a terminal to the given node. Additional
     calls will increase the weight from the given terminal to the
     node.  
  */
  void add_weight(int k, termtype terminal, tcaptype delta);

  /* Compute the maximum flow on the graph and return it. This
     performs the actual min-cut and modifies the graph to reflect
     that cut. After this call, one can use what_segment() to inspect
     which nodes are attached to which terminals.
  */
  flowtype maxflow();

  /* If there was a problem computing the maximal flow, this function
     will return an error. This function is an addition to the
     interface from the original implementation, which had no error
     handling capabilities.  
  */
  bool error();

  /* After the maximum flow has been computed on this graph,
     what_segment() allows the associated terminal for each node to be
     determined.  
  */
  termtype what_segment(int k);

private:
  /* Implementation notes: much of the design of this implementation
     was to minimize the amount of memory used on large graphs, as
     when image sizes hit 1M pixels, graph centric algorithms tend to
     use incredible amounts of space. Hence, we attempt to store
     things compactly without paying a large runtime overhead.

     The sets S, and A are represented by bit vectors, and an
     additional Alist is a list of active nodes in A that should be
     inspected.

     All edges are assumed to be bidirectional, which saves on memory
     as they only need to be created and maintained once. This also
     makes the augmentation phase very efficient.

     Edges from the terminals to the nodes are represented in the
     nodes themselves in a much more compact form. This saves a great
     deal of space since all terminal edges can only be
     unidirectional. This adds some complexity to the code, but turns
     out to provide a decent memory savings as well as much greater
     efficiency.

     Because grow() and adopt() work on variable lists (A and O
     respectively) which need to grow, we allocate links for the Alist
     and Olist in blocks to better pack memory, which leads to lower
     construction overhead as well as better caching performance
     because of the locality of the links. 
  */
  struct node;

  /* Our bidirectional edge structure. It's an edge from pfrom->pto
     with capacity cap, as well as a reverse edge between pto->pfrom
     with capacity rev_cap. Since this gets rather confusing in the
     code it's best to use the edge_* accessor functions which allow
     you to nod worry about which direction an edge is going and what
     it's capacity is.  
  */
  typedef struct {
    ncaptype cap_left;
    ncaptype rev_cap_left;

    struct node * pto;
    struct node * pfrom;
  } edge_t;


  /* Our node structure. It's a list of edges (since they're
     bidirectional, it's both the inbound and the outbound edges) and
     the capacity to both of the terminals. It also contains a
     parent_edge which is the edge that is used to connect to the
     current parent of this node in the graph. The parent edge can be
     null throughout portions of the algorithm or if the node is not
     in S.  
  */
  typedef struct node {
    int nedges;
    edge_t ** pedges;

    tcaptype cap_left[2];

    edge_t * pparent_edge;
  } node_t;


  /* Add an edge to the given node [this edge must be an inbound /
     outbound edge to this node.] Returns true if successful and false
     if it couldn't allocate memory.
  */
  __inline bool add_edge(node_t * pnode, edge_t * pedge)
  {
    // a little optimization for the image case. Most images have 4
    // edges, one for each neighbor on the grid. This optimization
    // helps make that the common case.  if we're not doing the normal
    // image case, just grow by 4. It's better than growing by 1.
    if ((pnode->nedges % 4) == 0){
      int newsize = (pnode->nedges + 4) * sizeof(edge_t *);
      
      // yes, this is C++, but I want to use malloc so I can realloc.
      pnode->pedges = (edge_t **)realloc(pnode->pedges, newsize);
      if (pnode->pedges == NULL){
        return false;
      }
    }

    pnode->pedges[pnode->nedges] = pedge;
    pnode->nedges++;

    return true;
  };

  /* Given a node, determines if it's source is the origin (if it
     originates at s) by tracking it's parent_edges backwards. Returns
     true if this node is connected to the source, false otherwise.
  */
  __inline bool origin_is_source(node_t * q)
  {
    // we check until our parent is null.
    while (q->pparent_edge != NULL){
      q = edge_src(q->pparent_edge, q);
    }

    // does this node have any capacity to the source?
    return (q->cap_left[SOURCE] > 0);
  }
  
  /* Get the node id for use as an index in the S and A bitvectors.
   */
  __inline int node_id(node_t * pnode)
  {
    return (pnode - pnodes);
  };


  /* We block allocate links for used in the lists of active and
     orphan nodes to improve memory packing and locality.  This
     constant sets how big the block size should be, as too big a
     block size wastes space, and too small a block size increases the
     fragmentation of memory and lowers the efficiency of getting new
     links.
  */
#define LINK_BLOCK_SIZE 512

  /* A node link for building the active and orphan lists. Has a next
     in the list [null if it's the last node] and the node of the
     link.
  */
  typedef struct node_link {
    node_t * pnode;
    struct node_link * pnext;
  } node_link_t;

  /* A block of node links. These are chained together as well into a
     list so they can be freed when the graph is destroyed.
   */
  typedef struct node_link_block {
    struct node_link_block * pnext;

    node_link_t plinks[LINK_BLOCK_SIZE];
  } node_link_block_t;

  /* We keep around the list of blocks of links we've allocated. The
     first one in the list is the most recent and may or may not have
     some links it hasn't given out yet.
  */
  node_link_block_t * pblocks;

  /* Counter for how many links the first block in the block list has
     left to give away. All subsequent blocks in the list have 0 links
     left. This implies that the first link to be given out (if
     nlinksLeft != 0) is pblocks->plinks[LINK_BLOCK_SIZE - nlinksLeft]
  */
  int nlinksLeft;

  /* We also keep a freed link list which is where we put links when
     they are freed from a list. This is the first place one should
     look when allocating new links.
  */
  node_link_t * pfreeLinks;


  /* Get a new node link, and initialize it. Returns the link or NULL
     if allocation failed.
  */
  __inline  node_link_t * new_node_link(node_t * pnode, node_link_t * pnext)
  {
    node_link_t * plink = NULL;
    // are there any free links?
    if (pfreeLinks == NULL){
      // no free links, but do we have any left in this block?
      if (nlinksLeft == 0){
        // guess not. Make a new block and return the first one.
        node_link_block_t * pblock = new node_link_block_t;
        if (pblock == NULL){
          return NULL;
        }

        pblock->pnext = pblocks;
        pblocks = pblock;

        nlinksLeft = LINK_BLOCK_SIZE - 1;
        plink = &(pblock->plinks[0]);
      }
      else {
        // else just give one of the links that are left.
        plink = &(pblocks->plinks[LINK_BLOCK_SIZE - nlinksLeft]);
        nlinksLeft--;
      }
    }
    else {
      // just give back the first one of the freelist.
      plink = pfreeLinks;
      pfreeLinks = pfreeLinks->pnext;
    }

    plink->pnext = pnext;
    plink->pnode = pnode;
    return plink;
  };

  /* Free a node link. Puts it on the freelist for use by another
     iteration.
   */
  __inline void free_node_link(node_link_t * plink)
  {
    plink->pnode = NULL;
    plink->pnext = pfreeLinks;
    pfreeLinks = plink;
  };

  /* Create a new edge. One could see blocking the edges like the node
     links are, but this is a far less dynamic event than allocating a
     node link though. I didn't feel it was worth the effort.

     This creates a new edge from pfrom->pto with capacity cap, and a
     reverse edge from pto->pfrom with a capacity rev_cap.
  */
  __inline edge_t * new_edge(ncaptype cap, ncaptype rev_cap, 
                             node_t * pfrom, node_t * pto)
  {
    edge_t * pedge = new edge_t();
    pedge->pto = pto;
    pedge->pfrom = pfrom;
    pedge->cap_left = cap;
    pedge->rev_cap_left = rev_cap;
    
    return pedge;
  }

  /* Free an edge.
   */
  __inline void free_edge(edge_t * pedge)
  {
    delete pedge;
  };

  /* Edge manipulation functions. The preferred way to play around
     with edges.
   */

  /* Given an edge and a node that is the destination of the edge, who
     is the source?
   */
  __inline node_t * edge_src(edge_t * pedge, node_t * dest)
  {
    return (pedge->pto == dest ? pedge->pfrom : pedge->pto);
  };

  /* Given an edge and a node that is the source, who is the
     destination?
   */
  __inline node_t * edge_dest(edge_t * pedge, node_t * src)
  {
    return (pedge->pfrom == src ? pedge->pto : pedge->pfrom);
  };

  /* Given an edge and a node that is the source, what is the capacity
     on that edge?  returns a reference so it can be modified. [note
     the reverse capacity is not modified when this capacity is
     modified.]
  */
  __inline ncaptype & edge_cap(edge_t * pedge, node_t * src)
  {
    return (pedge->pfrom == src ? pedge->cap_left : pedge->rev_cap_left);
  };

  /* Perform one iteration of the algorithm. Returns true if it is not
     finished and should be repeated, and false otherwise.
  */
  bool single_iter();

  /* grow() performs the growth phase of the algorithm and returns the
     node which is the sink's parent or null if no such path could be
     found.
  */
  node_t * grow();

  /* augment() pushes flow through the augmenting path found from
     grow(), and orphans some nodes into the orphan list Olist.
  */
  void augment(node_t * sink);

  /* adopt() clears out the orphan list and reforms the search tree,
     attempting to find parents for all the orphan nodes, and
     returning those who can't find parents into T.
  */
  void adopt();

  /* process() is the actual functionality of adopt() and processes a
   single node to find it a parent if possible.
   */
  void process(node_t * p);


  /* We keep a bitvector of n+1 entries which tells us for each node
     (and t), which nodes are currently in S. At the end of maxflow(),
     this bitvector tells us which nodes are in S, and which nodes are
     in T.
  */
  BitVector S;

  /* Our encapsulation of the actives set A is a little more
     complicated for efficiency reasons. We keep the Alist, which is a
     list of all the nodes in A, but we also need efficient mechanisms
     to remove an element from the Alist, as well as the ability to
     not introduce duplicates into the Alist. Hence, we keep A, which
     tells us for each node if it's in the Alist or not. If a node is
     found in the Alist whose bit is not set in A, it should be
     discarded because it has been removed from A but we didn't do the
     linear search to find it in the Alist.
  */
  BitVector A;
  node_link_t * Alist;
  node_link_t * lastA;
  
  /* The current orphan list. Will be NULL during the growth stage.
   */
  node_link_t * Olist;

  /* Our nodes. 
   */
  node_t * pnodes;
  int nnodes;

  /* Since we don't explicitely keep s or t as nodes, we need some way
     to refer to t.  We use tidx as it's id to tell if it's in S or A,
     and we keep around t's parent if it would have one.
  */
  int tidx;
  node_t * ptparent;

  /* Our current flow value. Once the growth phase does not find a
     path from s to t, this value will be the maximal flow in the
     graph.
  */
  flowtype cut_val;

  /* Was there an error computing the flow?
   */
  bool berror;
};

#endif
