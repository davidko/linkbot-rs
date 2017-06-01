# linkbot-rs

This library can be used to control Linkbots. Linkbots are small, modular
robots that are used in classrooms worldwide to teach programming, math,
science, and more. More information regarding Linkbots can be found
at https://barobo.com .

[![Crates.io](https://img.shields.io/crates/v/linkbot.svg)](https://crates.io/crates/linkbot)

[Documentation](https://docs.rs/linkbot)

## Usage

First, you will need `liblinkbot` installed on your current machine. For
Ubuntu, you can install `liblinkbot` with the following commands:

```text
wget http://repo.barobo.com/barobo.public.key -O - | sudo apt-key add -
sudo add-apt-repository "deb http://repo.barobo.com/ xenial main"
sudo apt-get update
sudo apt-get install liblinkbot
```

For Raspbian,
```text
wget http://barobo.com:81/barobo.public.key -O - | sudo apt-key add -
echo 'deb http://barobo.com:81/ jessie main' | sudo tee --append /etc/apt/sources.list
sudo apt-get update
sudo apt-get install liblinkbot
```

Next, add this to your `Cargo.toml`:
```toml
[dependencies]
linkbot = "*"
```

Then, add this to your crate:
```rust
extern crate linkbot;

use linkbot::Linkbot;
```

# License
`linkbot-rs` is distributed under the terms of GPL-3.0.
