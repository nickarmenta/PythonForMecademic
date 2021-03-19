FROM python:3.6-alpine

RUN apk add --no-cache git

COPY ./requirements.txt /requirements.txt

RUN pip install -r /requirements.txt
RUN pip install git+https://github.com/Mecademic/python_driver

COPY ./test.py /test.py

CMD ["python","/test.py"]
