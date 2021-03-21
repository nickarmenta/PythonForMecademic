FROM python:3.6-alpine

RUN apk add --no-cache git

WORKDIR /
RUN git clone https://github.com/nickarmenta/PythonForMecademic
WORKDIR /PythonForMecademic

RUN pip install -r /PythonForMecademic/requirements.txt
RUN python /PythonForMecademic/setup.py

CMD ["python","/test.py"]
