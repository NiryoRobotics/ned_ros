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

    def rm_all_since_date(self, date):
        query = 'DELETE FROM log WHERE date < :date'
        self.__dao.execute(query, {'date': date})
