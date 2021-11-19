import uuid


class Logs:
    def __init__(self, dao):
        self.__dao = dao

    def add(self, date, severity, origin, message):
        query = 'INSERT INTO log VALUES (:id, :date, :severity, :origin, :message)'
        self.__dao.execute(
            query, {
                'id': str(uuid.uuid4()),
                'date': date,
                'severity': severity,
                'origin': origin,
                'message': message
            }
        )

    def get_all(self):
        query = 'SELECT id, date, severity, origin, message FROM log'
        result = self.__dao.execute(query).fetchall()
        return result

    def rm_with_ids(self, ids):
        query = 'DELETE FROM log WHERE id in ({})'.format(
            ','.join('?' * len(ids))
        )
        self.__dao.execute(query, ids)
