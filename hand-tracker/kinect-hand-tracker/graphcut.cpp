#include <stdlib.h>
#include <stdio.h>

#include "graphcut.h"

BKCutGraph::BKCutGraph(int n) 
  // we allocate one more to the bit vectors for the status of t.
  : S(n+1), A(n+1)
{
  // create our list of nodes.
  nnodes = n;
  pnodes = (node_t *)calloc(sizeof(node_t), n);
 
  // active list and orphan lists are null.
  Alist = NULL;
  lastA = NULL;
  Olist = NULL;
  
  // the index we should use for t in A and S.
  tidx = n;

  // currently no link blocks allocated.
  pblocks = NULL;
  nlinksLeft = 0;
  pfreeLinks = NULL;

  reset();
}
	
BKCutGraph::~BKCutGraph()
{
  reset();
  
  free(pnodes);
  // free all the link blocks.
  node_link_block_t * pblock = pblocks;
  while (pblock != NULL){
    node_link_block_t * pnext = pblock->pnext;
    
    delete pblock;

    pblock = pnext;
  }
}


void BKCutGraph::reset()
{
  // clear all the nodes.

  // first step, clear out all the edges. We do this in 2 passes
  // because we don't want to free an edge twice, and we can't touch
  // them after we've free'd em. So we'll run through all the nodes,
  // NULL out every edge whose pto is this node [the other guy will
  // free em.]
  int i;
  for (i = 0 ; i < nnodes ; i++){
    node_t * pnode = &(pnodes[i]);
    for (int j = 0 ; j < pnode->nedges ; j++){
      edge_t * pedge = pnode->pedges[j];

      if (pedge->pto == pnode){
        pnode->pedges[j] = NULL;
      }
    }
  }

  // second pass does all the work and clears out the nodes.
  for (i = 0 ; i < nnodes ; i++){
    // free all the edges. In order not to free an edge 2x, we'll set
    // the entry in the other node to null when we delete it.
    node_t * pnode = &(pnodes[i]);
    for (int j = 0 ; j < pnode->nedges ; j++){
      edge_t * pedge = pnode->pedges[j];

      if (pedge != NULL){
        // null it out in the other node.
        delete pedge;
      }
    }
    free(pnode->pedges);

    pnode->pedges = NULL;
    pnode->nedges = 0;
    pnode->pparent_edge = NULL;

    pnode->cap_left[SOURCE] = 0;
    pnode->cap_left[SINK] = 0;
  }

  S.reset();
  A.reset();

  // anything that's on the Olist or the Alist should be added to the
  // pfreeLinks.
  node_link_t * plink = Alist;
  while (plink != NULL){
    node_link_t * pnext = plink->pnext;

    free_node_link(pnext);

    plink = pnext;
  }
  Alist = NULL;
  lastA = NULL;

  plink = Olist;
  while (plink != NULL){
    node_link_t * pnext = plink->pnext;

    free_node_link(pnext);

    plink = pnext;
  }
  Olist = NULL;

  ptparent = NULL;
  cut_val = 0;

  berror = false;
}

BKCutGraph::flowtype BKCutGraph::maxflow()
{
  while (single_iter());

  return cut_val;
}

bool BKCutGraph::error()
{
  return berror;
}

BKCutGraph::termtype BKCutGraph::what_segment(int k)
{
  // the segment that the node is in is just it's membership in S or
  // not.
  return (S.bit_value(k) ? SOURCE : SINK);
}

void BKCutGraph::add_arc(int from, int to, ncaptype cap)
{
  // an arc has a reverse capacity of 0.
  edge_t * pedge = new_edge(cap, 0, &(pnodes[from]), &(pnodes[to]));

  if (pedge == NULL){
    berror = true;
    return;
  }

  // add it to both the inbound and the outbound nodes. 
  add_edge(&(pnodes[from]), pedge);
  add_edge(&(pnodes[to]), pedge);
}

void BKCutGraph::add_edge(int from, int to, ncaptype cap, ncaptype cap_rev)
{
  // create the edge and add it to both nodes.
  edge_t * pedge = new_edge(cap, cap_rev, &(pnodes[from]), &(pnodes[to]));
  if (pedge == NULL){
    berror = true;
    return;
  }

  add_edge(&(pnodes[from]), pedge);
  add_edge(&(pnodes[to]), pedge);
}

void BKCutGraph::add_weight(int k, termtype terminal, tcaptype delta)
{
  // we add the capacity to the given terminal, and if it's nonzero to
  // the source, we add it to the actives list, essentially performing
  // the first step of the first growth stage here.
  node_t * pnode = &(pnodes[k]);
  pnode->cap_left[terminal] += delta;

  if (terminal == SOURCE){
    // add it to the S & A if it's not already.
    if ((pnode->cap_left[terminal] > 0) && (A.bit_value(node_id(pnode)) == 0)){
      A.bit_set(node_id(pnode));
      S.bit_set(node_id(pnode));
      node_link_t * plink = new_node_link(pnode, NULL);

      if (plink == NULL){
        berror = true;
        return;
      }

      if (Alist == NULL){
        Alist = plink;
      }
      else {
        lastA->pnext = plink;
      }

        lastA = plink;
    }
    else if (pnode->cap_left[terminal] <= 0){
      S.bit_clear(node_id(pnode));
      A.bit_clear(node_id(pnode));
    }
  }
}

bool BKCutGraph::single_iter()
{
  node_t * sink = grow();
  if (sink == NULL){
    return false;
  }

  augment(sink);
  adopt();
  return true;
}


BKCutGraph::node_t * BKCutGraph::grow()
{
  // if t is in S, return t's parent..
  if (S.bit_value(tidx)){
    return ptparent;
  }

  while (Alist != NULL){
    // pick an active node in A, and see if it can grow..
    node_link_t * plink = Alist;
    node_t * p = plink->pnode;

    // make sure it's really in A. Needs to be in A and the Alist, if
    // not, just ditch it.
    if (A.bit_value(node_id(p)) == 0){
      Alist = Alist->pnext;
      free_node_link(plink);
      continue;
    }

    // We'll be proactive and find short paths, does this node have
    // any capacity to t? If so, we're done.
    if (p->cap_left[SINK] > 0){
      S.bit_set(tidx);
      ptparent = p;
      return ptparent;
    }

    // No capacity to t. We'll see if we can grow.  for each edge.
    for (int i = 0 ; i < p->nedges ; i++){
      edge_t * pedge = p->pedges[i];
      node_t * q = edge_dest(pedge, p);

      // if it's not in S, and it's not saturated outbound, add q to
      // the tree as a active node.
      if ((S.bit_value(node_id(q)) == 0) && (edge_cap(pedge, p) > 0)){
        // S = S U {q}
        S.bit_set(node_id(q));

        // A = A U {q}
        if (A.bit_value(node_id(q)) == 0){
          node_link_t * pnewlink = new_node_link(q, NULL);

          if (pnewlink == NULL){
            berror = true;
            return NULL;
          }

          lastA->pnext = pnewlink;
          lastA = pnewlink;

          A.bit_set(node_id(q));
        }

        // PARENT(q) := p
        q->pparent_edge = pedge;
      }
    }

    // it can't grow anymore, so remove it from A.
    Alist = plink->pnext;
    free_node_link(plink);
    A.bit_clear(node_id(p));


  }

  lastA = NULL;
  // if there are no more active nodes and we haven't found it, we're
  // done.
  return NULL;
}

void BKCutGraph::augment(BKCutGraph::node_t * psink)
{
  // we've got a path from s->t, let's find the bottleneck capacity
  // and push as much through it as we can.
  ncaptype min_cap = psink->cap_left[SINK];
  node_t * pcur = psink;
  while (pcur->pparent_edge != NULL){
    node_t * pprev = edge_src(pcur->pparent_edge, pcur);
    ncaptype cap = edge_cap(pcur->pparent_edge, pprev);
    
    if (cap < min_cap){
      min_cap = cap;
    }
    pcur = pprev;
  }

  // don't forget to incorporate the capacity on the edge from the
  // source.
  if (pcur->cap_left[SOURCE] < min_cap){
    min_cap = pcur->cap_left[SOURCE];
  }

  // our max flow is this much greater.
  cut_val += min_cap;

  // update all edges on the path with the bottleneck capacity.

  // first the source.
  pcur->cap_left[SOURCE] -= min_cap;
  // does this orphan the node who's direct parent is the source?  if
  // it's saturated, put cur in the orphans list.
  if (pcur->cap_left[SOURCE] == 0){
    pcur->pparent_edge = NULL;

    // O = O U {cur}
    node_link_t * pnewlink = new_node_link(pcur, Olist);
    if (pnewlink == NULL){
      berror = true;
      return;
    }

    Olist = pnewlink; 
  }

  // update the capacity to the sink.
  psink->cap_left[SINK] -= min_cap;

  // ignore orphaning t; this is different than the algorithm in the
  // paper, but fine since we always grow to find t.
  if (psink->cap_left[SINK] == 0){
    ptparent = NULL;
    S.bit_clear(tidx);
  }

  // now update the capacity along the path, also pushing capacity in
  // the reverse direction.
  pcur = psink;
  while (pcur->pparent_edge != NULL){
    node_t * pprev = edge_src(pcur->pparent_edge, pcur);
    ncaptype & cap = edge_cap(pcur->pparent_edge, pprev);
    ncaptype & rev_cap = edge_cap(pcur->pparent_edge, pcur);

    cap -= min_cap;
    // push the opposite capacity the other way.
    rev_cap += min_cap;

    if (cap == 0){
      // if it's saturated towards the sink, put cur in the orphans
      // list.
      pcur->pparent_edge = NULL;

      // O = O U {cur}
      node_link_t * pnewlink = new_node_link(pcur, Olist);

      if (pnewlink == NULL){
        berror = true;
        return;
      }

      Olist = pnewlink; 
    }
    pcur = pprev;
  }
}


void BKCutGraph::adopt()
{
  // just cycle through all the orphans and process them.
  while (Olist != NULL){
    node_link_t * plink = Olist;
    node_t * p = plink->pnode;

    // remove p from O
    Olist = plink->pnext;
    free_node_link(plink);

    process(p);
  }
}


void BKCutGraph::process(BKCutGraph::node_t * p)
{
  // first try. Let's see if we can find a parent for p.  we need to
  // find an edge into p with some capacity whose source is in S, and
  // whose origin is S (it's not an orphan.)
  int i;
  for (i = 0 ; i < p->nedges ; i++){
    edge_t * pedge = p->pedges[i];
    node_t * q = edge_src(pedge, p);

    if ((edge_cap(pedge, q) > 0) 
        && (S.bit_value(node_id(q)) != 0)
        && (origin_is_source(q))){
      p->pparent_edge = pedge;
      return;
    }
  }
  
  // looks like we didn't find one.  remove p from S, A, and give all
  // it's children a chance to find new parents.
  S.bit_clear(node_id(p));
  A.bit_clear(node_id(p));

  // for all children q of p, PARENT(q) = NULL, and add them to the
  // orphans.
  for (i = 0 ; i < p->nedges ; i++){
    edge_t * pedge = p->pedges[i];
    node_t * q = edge_dest(pedge, p);

    if (q->pparent_edge == pedge){
      q->pparent_edge = NULL;

      // add it to the set of orphans.
      // O = O U {q}
      node_link_t * pnewlink = new_node_link(q, Olist);
      if (pnewlink == NULL){
        berror = true;
        return;
      }

      Olist = pnewlink;
    }
  }

  // if t is p's child, remove it from S too.
  if (ptparent == p){
    ptparent = NULL;
    S.bit_clear(tidx);
  }

  // all potential parents of p in S, are added to A so maybe they can
  // regrow and bring S back in t.
  for (i = 0 ; i < p->nedges ; i++){
    edge_t * pedge = p->pedges[i];
    node_t * q = edge_src(pedge, p);
    if ((edge_cap(pedge, q) > 0) 
        && (S.bit_value(node_id(q)) != 0) 
        && (A.bit_value(node_id(q)) == 0)){
      node_link_t * pnewlink = new_node_link(q, NULL);
      if (pnewlink == NULL){
        berror = true;
        return;
      }
      
      if (Alist == NULL){
        Alist = pnewlink;
        lastA = pnewlink;
      }
      else {
        lastA->pnext = pnewlink;
        lastA = pnewlink;
      }
      
      A.bit_set(node_id(q));
    }
  }
}

