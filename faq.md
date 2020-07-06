# FAQ

## How to clean all my modifications on docker to start on a clean setup again ?

```
docker volume rm summer-school
```

## How to get the latest version of the tutorials with Docker ?

```
docker run --rm -v summer-school:/home/student -it memoryofmotion/summer-school git -C summer-school pull --rebase --recurse-submodules --ff-only
```

## Docker: Ports already used

- Symptoms:
    - `Bind for 0.0.0.0:8888 failed: port is already allocated.`
    - `listen tcp 0.0.0.0:8888: bind: address already in use.`
- Solution:

    Another software is already using the port 8888 on your computer. This might be another docker container, please
    run `docker ps` to see if you have any container using `0.0.0.0:8888->8888/tcp` in the `PORTS` column. In this
    case, you can decide to stop it, and then try again.

    Otherwise, you can dedice to bind jupyter's port 8888 inside the container to another port on your computer, *eg.*
    8889: `docker run --rm -p 7000:7000 -p 8889:8888 -v summer-school:/home/student -it memoryofmotion/summer-school`.
    Then you will have to adapt the url given by jupyter with this port, *eg.* http://127.0.0.1:8889/?token=2fa…

    The same solution can be used for meshcat port 7000. In that case, to open meshcat, you will have to use *eg.*
    http://127.0.0.1:7001/static/
