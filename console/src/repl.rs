
use core::marker::PhantomData;

use heapless;

const SIZE: usize = 10;
const STR: usize = 64;

pub type CommandFunc<'a, Error> = fn(heapless::FnvIndexMap<heapless::String<STR>, Value, SIZE>) -> Result<&'a str, Error>;

type String = heapless::String<STR>;

#[derive(Debug, Clone, Copy)]
enum BaseType {
    String,
    Integer,
    Float,
}

enum Error {
    ConversionError(String, BaseType),
}


trait Convert<T> {
    fn convert(&self) -> Result<T, Error>;
}

struct Value {
    typ: BaseType,
    term: heapless::String<STR>
}

impl Value {
    fn new(term: heapless::String<STR>, btype: BaseType) -> Value {
        Value { 
            typ: btype,
            term 
        }
    }
}

impl Convert<i32> for Value {
    fn convert(&self) -> Result<i32, Error> {
        match self.term.parse::<i32>() {
            Ok(n) => Ok(n),
            Err(_) => Err(Error::ConversionError(self.term.clone(), self.typ))
        }
    }
}

impl Convert<String> for Value {
    fn convert(&self) -> Result<String, Error> {
        Ok(self.term.clone())
    }
}

impl Convert<f32> for Value {
    fn convert(&self) -> Result<f32, Error> {
        match self.term.parse::<f32>() {
            Ok(f) => Ok(f),
            Err(_) => Err(Error::ConversionError(self.term.clone(), self.typ))
        }
    }
}


pub struct Command<'a, Error> {
    name: &'static str,
    parameters: heapless::Vec<(String, BaseType), SIZE>,
    command: CommandFunc<'a, Error>
}

fn _example_command<'a>(parameters: heapless::FnvIndexMap<heapless::String<STR>, Value, SIZE>) -> Result<String, Error> {
    // Expect single parameter named `name`
    let name: String = parameters[&"name".into()].convert()?;
    Ok(name)
}
