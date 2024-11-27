-- --------------------------------------------------------
-- 호스트:                          songhu.co.kr
-- 서버 버전:                        5.6.41 - MySQL Community Server (GPL)
-- 서버 OS:                        Linux
-- HeidiSQL 버전:                  12.8.0.6908
-- --------------------------------------------------------

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET NAMES utf8 */;
/*!50503 SET NAMES utf8mb4 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

-- 테이블 siptdb.ADMIN_COOKIE 구조 내보내기
CREATE TABLE IF NOT EXISTS `ADMIN_COOKIE` (
  `ADMIN_ID` varchar(50) NOT NULL DEFAULT '',
  `AUTO_LOGIN_KEY` varchar(50) NOT NULL DEFAULT '',
  `REMOTE_ADDR` varchar(50) DEFAULT NULL,
  `REG_DT` datetime DEFAULT NULL,
  PRIMARY KEY (`ADMIN_ID`,`AUTO_LOGIN_KEY`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;

-- 내보낼 데이터가 선택되어 있지 않습니다.

-- 테이블 siptdb.ADMIN_MST 구조 내보내기
CREATE TABLE IF NOT EXISTS `ADMIN_MST` (
  `ADMIN_ID` varchar(50) NOT NULL DEFAULT '',
  `ADMIN_NM` varchar(50) DEFAULT NULL,
  `TEL_NO` varchar(50) DEFAULT NULL,
  `EMAIL` varchar(255) DEFAULT NULL,
  `AUTO_LOGIN` int(11) DEFAULT NULL,
  `PASSWD` varchar(100) DEFAULT NULL,
  `AUTH_LEVEL` varchar(50) DEFAULT NULL,
  `REG_DT` datetime DEFAULT NULL,
  PRIMARY KEY (`ADMIN_ID`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;

-- 내보낼 데이터가 선택되어 있지 않습니다.

-- 테이블 siptdb.BOARD_ANN 구조 내보내기
CREATE TABLE IF NOT EXISTS `BOARD_ANN` (
  `SEQ_NO` bigint(20) NOT NULL AUTO_INCREMENT,
  `TITLE` varchar(255) DEFAULT NULL,
  `CONTENT` varchar(2000) DEFAULT NULL,
  `DEL_YN` int(11) DEFAULT '0',
  `VALID_DT` datetime DEFAULT NULL,
  `REG_DT` datetime DEFAULT NULL,
  `MOD_DT` datetime DEFAULT NULL,
  `REG_ID` bigint(20) DEFAULT NULL,
  `IS_POPUP` int(11) DEFAULT NULL,
  `GAME_TYPE` varchar(20) DEFAULT NULL,
  `USER_ID` bigint(20) DEFAULT NULL,
  `ANN_TYPE` varchar(10) DEFAULT 'A',
  PRIMARY KEY (`SEQ_NO`)
) ENGINE=InnoDB AUTO_INCREMENT=535 DEFAULT CHARSET=utf8;

-- 내보낼 데이터가 선택되어 있지 않습니다.

/*!40103 SET TIME_ZONE=IFNULL(@OLD_TIME_ZONE, 'system') */;
/*!40101 SET SQL_MODE=IFNULL(@OLD_SQL_MODE, '') */;
/*!40014 SET FOREIGN_KEY_CHECKS=IFNULL(@OLD_FOREIGN_KEY_CHECKS, 1) */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40111 SET SQL_NOTES=IFNULL(@OLD_SQL_NOTES, 1) */;
