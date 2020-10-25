pub struct DoubleBuffer<T> {
    front: Vec<T>,
    back: Vec<T>,
}

impl<T: Clone> DoubleBuffer<T> {
    pub fn from(front: Vec<T>) -> Self { // TODO From trait?
        let back = front.clone();
        DoubleBuffer {front, back}
    }

    pub fn front(&self) -> &Vec<T> {
        &self.front
    }

    pub fn write(&mut self, i: usize, value: T) {
        self.back[i] = value;
    }

    pub fn read(&self, i: usize) -> T {
        self.front[i].clone()
    }

    pub fn flip(&mut self) {
        std::mem::swap(&mut self.front, &mut self.back);
    }
}
