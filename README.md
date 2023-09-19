[![Docker](https://github.com/kas-lab/metacontrol_knowrob/actions/workflows/docker-publish.yml/badge.svg)](https://github.com/kas-lab/metacontrol_knowrob/actions/workflows/docker-publish.yml)
# metacontrol_knowrob
Metacontrol implementation with the KnowRob framework

## Clone

```Bash
cd ~/
git clone git@github.com:kas-lab/metacontrol_knowrob.git
```

## Running it

In one terminal:
```Bash
cd ~/metacontrol_knowrob/
docker-compose up
```

In another terminal:
```Bash
docker run -it --rm --net knowrob_network --env ROS_MASTER_URI=http://knowrob:11311 ghcr.io/kas-lab/metacontrol_knowrob:main rosrun metacontrol_knowrob metacontrol.py
```

## Run tests

In one terminal:
```Bash
cd ~/metacontrol_knowrob/
docker-compose up
```

In another terminal:
```Bash
docker run -it --rm --net knowrob_network --env ROS_MASTER_URI=http://knowrob:11311 ghcr.io/kas-lab/metacontrol_knowrob:main python -m pytest src/metacontrol_knowrob/test/rules/test_rules.py
```

## Build docker image locally

In one terminal:
```Bash
cd ~/metacontrol_knowrob/
docker build -t metacontrol_knowrob .
```
Then use the `metacontrol_knowrob` image instead of `ghcr.io/kas-lab/metacontrol_knowrob:main`
