# Zenoh Router Setup

When communication has to be via the Internet, or a NAT is connected between the robot and the operator, the default middleware reaches its limits. For these use cases, Zenoh middleware with the associated Zenoh routers is a good choice.

Specifically, this setup helps when a Linux instance is running on a computer behind a NAT, such as with WSL2 or Docker Desktop. In this case, IT policies may prevent the NAT from being disabled.

## Setup

### On Robot

On the robot, the middleware only needs to be switched to Zenoh via an environment variable. When the Docker containers on the robot are restarted, they will use the newly set middleware, and a Zenoh router will also be started.

The easiest way to set the variable is via the environment file. Open the file by:

```bash
sudo nano /etc/environment
```

And set the **RMW_IMPLEMENTATION** variable to **rmw_zenoh_cpp**. Thats it!

### On Host (Operator)

On the host side, two configurations must be created and two additional environment variables must be set. In addition, the middleware must of course also be set to Zenoh.

>**Note:** below is just collected information in a not user friendly format. Sorry

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CONFIG_URI=zenoh_router_config.json5
export ZENOH_SESSION_CONFIG_URI=zenoh_session_config.json5
```

zenoh_router_config.json5
```json5
{
  mode: "router",

  listen: {
    endpoints: {
      router: ["tcp/0.0.0.0:7447"]
    }
  },
  connect: {
    endpoints: {
      router: ["tcp/192.168.23.126:7447"]
    }
  },

  // Experimental...
  scouting: {
    multicast: {
      enabled: true,
      address: "224.0.0.224:7446",
      interface: "auto"
    }
  },
  transport: {
    shared_memory: {
      enabled: true
    }
  }
}
```

zenoh_session_config.json5
```json5
{
  mode: "peer",
  connect: {
    // Ersetze IP und Port durch die Daten deines Routers
    endpoints: {
      router: ["tcp/127.0.0.1:7447"]
    }
  },

  // Experimental...
  scouting: {
    multicast: {
      enabled: true,
      address: "224.0.0.224:7446",
      interface: "auto"
    }
  },
  transport: {
    shared_memory: {
      enabled: true
    }
  }
}
```