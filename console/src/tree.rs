
use crate::hash::SDBMHasher;

use alloc::vec::Vec;
use core::{default::Default, hash::BuildHasherDefault, marker::PhantomData, slice};

use hashbrown::HashMap;

#[derive(Debug, Hash, Eq, PartialEq, Copy, Clone)]
pub struct NodeID(u32);

struct IDBuilder(u32);

impl IDBuilder {
    fn next(&mut self) -> NodeID {
        self.0 += 1;
        NodeID(self.0)
    }
}

impl Default for IDBuilder {
    fn default() -> IDBuilder {
        IDBuilder(0)
    }
}

#[derive(Debug)]
pub enum Error {
    NoSuchParent,
}

pub struct Tree<T> {
    nodes: HashMap<NodeID, T, SDBMHasher>,
    children: HashMap<NodeID, Vec<NodeID>, BuildHasherDefault<SDBMHasher>>,
    id_builder: IDBuilder
}

impl<T> Tree<T> {
    pub fn new() -> Tree<T> {
        Tree {
            nodes: HashMap::default(),
            children: HashMap::default(),
            id_builder: IDBuilder::default()
        }
    }

    pub fn add_node(&mut self, node: T) -> NodeID {
        let id = self.id_builder.next();
        let _ = self.nodes.insert(id, node);
        id
    }

    pub fn add_child(&mut self, parent: &NodeID, child: T) -> Result<NodeID, Error> {
        if !self.nodes.contains_key(parent) {
            Err(Error::NoSuchParent)
        } else {
            let child_id = self.add_node(child);

            self.children.entry(*parent)
                .or_insert(Vec::new())
                .push(child_id);

            Ok(child_id)
        }
    }

    pub fn fetch(&self, node: &NodeID) -> Option<&T> {
        self.nodes.get(node)
    }

    pub fn children(&self, node: &NodeID) -> &[NodeID] {
        let children = self.children.get(node);
        match children {
            Some(ch) => ch.as_slice(),
            None => &[]
        }
    }

    pub fn child_iter(&self, node: &NodeID) -> ChildIterator<T> {
        let ch = self.children(node);
        ChildIterator::new(ch.iter(), &self)
    }

    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }
}

pub struct ChildIterator<'a, T> {
    iter: slice::Iter<'a, NodeID>,
    tree: &'a Tree<T>,
    _t: PhantomData<T>
}

impl<'a, T> ChildIterator<'a, T> {
    fn new(iter: slice::Iter<'a, NodeID>, tree: &'a Tree<T>) -> ChildIterator<'a, T> {
        ChildIterator {
            iter,
            tree,
            _t: PhantomData,
        }
    }
}

impl<'a, T: 'a> Iterator for ChildIterator<'a, T> {
    type Item = &'a T;

    fn next(&mut self) -> Option<&'a T> {
        self.iter.next().and_then(|id| self.tree.fetch(id))
    }
}
