use crate::tree::{NodeID, Tree};

use hifive1::sprintln;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Input {
    ESC,
    NewLine,
    Char(u8)
}

impl From<&str> for Input {
    fn from(c: &str) -> Input {
        match c {
            "" => Input::Char(0),
            "ESC" => Input::ESC,
            "\n" => Input::NewLine,
            _ => Input::Char(c.as_bytes()[0] as u8) // Only respect the first byte I guess?
        }
    }
}

impl From<u8> for Input {
    fn from(c: u8) -> Input {
        match c {
            27 => Input::ESC,
            13 | 10 => Input::NewLine,
            _ => Input::Char(c)
        }
    }
}

impl Default for Input {
    fn default() -> Input {
        Input::Char(0)
    }
}

impl From<Input> for char {
    fn from(i: Input) -> char {
        u8::from(i) as char
    }
}

impl From<Input> for u8 {
    fn from(i: Input) -> u8 {
        match i {
            Input::ESC => 27,
            Input::NewLine => 10,
            Input::Char(c) => c
        }
    }
}


#[derive(Debug, Clone, Copy)]
pub enum Output {
    DEL,
    HOME,
    F1,
    ESC,
    CursorRight(u8),
    CursorLeft(u8),
    CursorUp(u8),
    CursorDown(u8),
    INSERT,
    EraseFromCursor
}




// `ESC, [, 3, ~` => DEL
// `ESC, [, 1, ~` => HOME
// `ESC, [, 1, 1, ~` => F1
// `ESC, ESC` => ESC

/// Node has a Key and Map of the connected Node and an Action A to associated with the transition.
/// So, Node N has key k1, and is connected to Node with key k2. The transition from k1 to k2 has action a.
#[derive(Debug)]
enum Node<K, V> {
    Terminal(K, V),
    Node(K)
}


/// Represents the state of the control sequence parser. If the Machine
/// is in the middle of detecting a control sequence, then that is `InSequence`.
/// Otherwise if the Machine is receiving normal input, this is the `Listening` state.
pub enum MachineState<T> {
    InSequence(T),
    Listening(T)
}

pub struct Trie<K: Default + Copy + PartialEq + core::fmt::Debug, V> {
    current_state: NodeID,
    root: NodeID,
    tree: Tree<Node<K, V>>
}

impl<K: Default + Copy + PartialEq + core::fmt::Debug, V: Copy> Trie<K, V> {
    pub fn new() -> Trie<K, V> {
        let root = K::default();
        let mut t = Tree::new();
        let rid = t.add_node(Node::Node(root));

        Trie {
            current_state: rid,
            root: rid,
            tree: t
        }
    }

    pub fn from_slices(slices: &[ (&[K], V) ]) -> Trie<K, V> {
        let mut t = Trie::new();
        for (s, v) in slices.into_iter() {
            // Dereferencing v here works bc we said above that V is Copy
            t.insert(*s, *v);
        }
        t
    }

    pub fn insert_iter<It: IntoIterator<Item=K>>(&mut self, sequence: It, value: V) {
        let mut seq_iter = sequence.into_iter();
        // Keep track of two arrows into the iterator, current and on_deck.
        // current is the key element being processed now. on_deck is what's coming next.
        let mut current = match seq_iter.next() {
            Some(k) => k,
            None => { return } // Bail early if we have no items
        };
        let mut on_deck = seq_iter.next();

        /// In the iterator, the current element is last if
        /// `on_deck` is None
        fn is_last<K>(on_deck: Option<K>) -> bool {
            on_deck.is_none()
        }

        let mut vid = self.root;

        while !is_last(on_deck) {
            // who are vid's neighbors, and if current == the value of a neighbor, then we've already added current
            let mut already_found = false;
            for neighbor in self.tree.children(&vid) {
                if let Some(n_node) = self.tree.fetch(neighbor) {
                    match n_node {
                        Node::Node(nj) => {
                            if &current == nj {
                                // Here we would like to move on to the next item in the input sequence
                                already_found = true;
                                // New starting place is the ID of the neighbor we matched keys with
                                vid = *neighbor;
                            }
                        },
                        _ => {}
                    }
                } 
            }

            if !already_found {
                let id = self.tree.add_child(&vid, Node::Node(current)).unwrap();
                // Switch current vid with the id we just added
                vid = id;
            }

            // We can unwrap since the None case is guarded against in the while loop
            // Current becomes what was the upcoming `on_deck`. Grab the next item `on_deck`
            current = on_deck.unwrap();
            on_deck = seq_iter.next();
        }
        // When we get here, we know current is the last element since on_deck is None
        let _ = self.tree.add_child(&vid, Node::Terminal(current, value));
    }

    pub fn insert(&mut self, sequence: &[K], value: V) {
        self.insert_iter(sequence.iter().copied(), value);
    }

    pub fn input_next(&mut self, key: K) -> (Option<&V>, MachineState<K>) {
        for n in self.tree.children(&self.current_state) {
            if let Some(node) = self.tree.fetch(n) {
                match node {
                    Node::Node(k) => {
                        if &key == k {
                            // If the input key matches a neighbor node key, then we've found the next step down the tree
                            // So put this VertexId as the current_state and we're done.
                            // VertexId is Copy, so dereferencing here is fine, we don't move the value.
                            self.current_state = *n;
                            return (None, MachineState::InSequence(key));
                        }
                    },
                    Node::Terminal(k, v) => {
                        if &key == k {
                            // If the input key matches this neighbor Terminal node, then we found our final value.
                            // Return the V, and reset the state to the root VetexId
                            self.current_state = self.root;
                            return (Some(v), MachineState::InSequence(key));
                        }
                    }
                }
            }
        }
        // We got to the end of the neighbors without finding anything
        // This means we do not have a valid sequence and so the state should be reset
        // and we should also return None
        self.current_state = self.root;
        (None, MachineState::Listening(key))
    }
}

impl Trie<Input, Output> {
    pub fn from_strs(slices: &[ (&[&str], Output) ]) -> Trie<Input, Output> {
        let mut t = Trie::new();
        for (seq, v) in slices {
            let it = (*seq).into_iter().map(|s| Input::from(*s));
            t.insert_iter(it, *v);
        }
        t
    }
}
